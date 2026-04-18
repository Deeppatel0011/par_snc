#!/usr/bin/env python3

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String
import tf2_ros

from par_snc.common import STATUS_TOPIC


def _yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class PathTrackerReturnHome(Node):
    def __init__(self):
        super().__init__('node3_path_tracker_return_home')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sample_period', 0.5)
        self.declare_parameter('sample_distance', 0.15)
        self.declare_parameter('waypoint_stride', 1)
        self.declare_parameter('min_return_waypoint_distance', 0.25)
        self.declare_parameter('skip_close_waypoint_distance', 0.35)
        self.declare_parameter('debug_log_paths', False)
        self.declare_parameter('debug_log_every_n', 1)
        self.declare_parameter('navigate_to_pose_topic', '/navigate_to_pose')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sample_period = float(self.get_parameter('sample_period').value)
        self.sample_distance = float(self.get_parameter('sample_distance').value)
        self.waypoint_stride = max(1, int(self.get_parameter('waypoint_stride').value))
        self.min_return_waypoint_distance = float(
            self.get_parameter('min_return_waypoint_distance').value
        )
        self.skip_close_waypoint_distance = float(
            self.get_parameter('skip_close_waypoint_distance').value
        )
        self.debug_log_paths = bool(self.get_parameter('debug_log_paths').value)
        self.debug_log_every_n = max(1, int(self.get_parameter('debug_log_every_n').value))
        self.navigate_to_pose_topic = self.get_parameter('navigate_to_pose_topic').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_explore_pub = self.create_publisher(Path, '/path_explore', 10)
        self.path_return_pub = self.create_publisher(Path, '/path_return', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)

        self.create_subscription(Empty, '/trigger_start', self._on_start, 10)
        self.create_subscription(Empty, '/trigger_home', self._on_return_home, 10)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)

        self.path_explore = Path()
        self.path_explore.header.frame_id = self.map_frame

        self.path_return = Path()
        self.path_return.header.frame_id = self.map_frame

        self.exploration_started = False
        self.return_active = False
        self.return_waypoints = []
        self.return_waypoint_index = 0
        self.explore_publish_count = 0
        self.return_publish_count = 0

        self.action_client = ActionClient(self, NavigateToPose, self.navigate_to_pose_topic)
        self.active_goal_handle = None

        self.timer = self.create_timer(self.sample_period, self._track_robot_pose)
        self._publish_status('waiting_for_start')

        self.get_logger().info('Node 3 ready: tracking path and return-home logic enabled.')

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_start(self, _msg):
        if self.exploration_started:
            return
        self.exploration_started = True
        self.return_active = False
        self._publish_status('exploring')
        self.get_logger().info('Received /trigger_start. Exploration path tracking started.')

    def _on_return_home(self, _msg):
        if self.return_active:
            self.get_logger().warn('Return-home already active. Ignoring duplicate trigger.')
            return

        if len(self.path_explore.poses) < 2:
            self.get_logger().warn('Not enough exploration points to return home.')
            self._publish_status('return_failed_not_enough_points')
            return

        self.return_active = True
        self._publish_status('return_home_requested')
        self.path_return.poses = []

        reverse_points = list(reversed(self.path_explore.poses))
        sparse_reverse = reverse_points[::self.waypoint_stride]

        current_pose = self._lookup_current_pose()
        if current_pose is None:
            self.get_logger().warn('Cannot lookup current pose, using unfiltered reverse path.')
            filtered_waypoints = sparse_reverse
        else:
            filtered_waypoints = []
            for waypoint in sparse_reverse:
                if self._distance_between_pose_stamped(current_pose, waypoint) < self.skip_close_waypoint_distance:
                    continue

                if filtered_waypoints and (
                    self._distance_between_pose_stamped(filtered_waypoints[-1], waypoint)
                    < self.min_return_waypoint_distance
                ):
                    continue

                filtered_waypoints.append(waypoint)

        self.return_waypoints = filtered_waypoints if filtered_waypoints else sparse_reverse

        # Ensure the final start point exists as last target.
        if self.return_waypoints[-1] != self.path_explore.poses[0]:
            self.return_waypoints.append(self.path_explore.poses[0])

        self.return_waypoint_index = 0
        self.get_logger().info(
            f'Received /trigger_home. Starting reverse path with {len(self.return_waypoints)} waypoints.'
        )
        self._send_next_waypoint_goal()

    def _on_status(self, msg):
        status = msg.data.strip().lower()

        # Support Node 1 status-driven behavior in addition to trigger topics.
        if ('exploring frontiers' in status or 'navigating to frontier' in status) and not self.exploration_started:
            self.get_logger().info('Status indicates exploration started. Enabling path tracking.')
            self._on_start(None)
            return

        if 'returning home' in status and not self.return_active:
            self.get_logger().info('Status indicates return-home started. Replaying reverse path.')
            self._on_return_home(None)

    def _track_robot_pose(self):
        if not self.exploration_started and not self.return_active:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().debug(f'TF lookup failed: {ex}')
            return

        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = now
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation

        if self.return_active:
            if self._should_append_pose(self.path_return, pose):
                self.path_return.poses.append(pose)
                self.return_publish_count += 1
                self._log_path_event('path_return', self.return_publish_count, pose, len(self.path_return.poses))
            self.path_return.header.stamp = now
            self.path_return_pub.publish(self.path_return)
            return

        if self._should_append_pose(self.path_explore, pose):
            self.path_explore.poses.append(pose)
            self.explore_publish_count += 1
            self._log_path_event('path_explore', self.explore_publish_count, pose, len(self.path_explore.poses))

        self.path_explore.header.stamp = now
        self.path_explore_pub.publish(self.path_explore)

    def _should_append_pose(self, path_msg, new_pose):
        if not path_msg.poses:
            return True

        last = path_msg.poses[-1].pose.position
        cur = new_pose.pose.position
        distance = math.hypot(cur.x - last.x, cur.y - last.y)
        return distance >= self.sample_distance

    def _lookup_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except tf2_ros.TransformException:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _distance_between_pose_stamped(pose_a, pose_b):
        dx = pose_a.pose.position.x - pose_b.pose.position.x
        dy = pose_a.pose.position.y - pose_b.pose.position.y
        return math.hypot(dx, dy)

    def _log_path_event(self, topic_name, event_index, pose, total_points):
        if not self.debug_log_paths:
            return
        if event_index % self.debug_log_every_n != 0:
            return

        self.get_logger().info(
            f'[{topic_name}] event={event_index} total_points={total_points} '
            f'x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f}'
        )

    def _send_next_waypoint_goal(self):
        if not self.return_active:
            return

        if self.return_waypoint_index >= len(self.return_waypoints):
            self.return_active = False
            self._publish_status('home_reached')
            self.get_logger().info('Return-home complete.')
            return

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose action server unavailable.')
            self._publish_status('return_failed_no_action_server')
            self.return_active = False
            return

        target_pose = self.return_waypoints[self.return_waypoint_index]
        goal_pose = PoseStamped()
        goal_pose.header = target_pose.header
        goal_pose.pose = target_pose.pose

        # Minimal spin fix: face each waypoint in the direction of the next segment.
        if len(self.return_waypoints) > 1:
            if self.return_waypoint_index < (len(self.return_waypoints) - 1):
                next_pose = self.return_waypoints[self.return_waypoint_index + 1]
            else:
                next_pose = self.return_waypoints[self.return_waypoint_index - 1]

            dx = next_pose.pose.position.x - target_pose.pose.position.x
            dy = next_pose.pose.position.y - target_pose.pose.position.y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                yaw = math.atan2(dy, dx)
                qx, qy, qz, qw = _yaw_to_quaternion(yaw)
                goal_pose.pose.orientation.x = qx
                goal_pose.pose.orientation.y = qy
                goal_pose.pose.orientation.z = qz
                goal_pose.pose.orientation.w = qw

        if self.debug_log_paths:
            self.get_logger().info(
                '[return_goal] '
                f'idx={self.return_waypoint_index + 1}/{len(self.return_waypoints)} '
                f'x={goal_pose.pose.position.x:.3f} y={goal_pose.pose.position.y:.3f}'
            )
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._publish_status(f'returning_waypoint{self.return_waypoint_index + 1}')
        send_goal_future = self.action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint goal rejected. Trying next waypoint.')
            self.return_waypoint_index += 1
            self._send_next_waypoint_goal()
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result()
        status = result.status
        status_label = self._goal_status_to_text(status)

        if status != GoalStatus.STATUS_SUCCEEDED:
            nav_result = result.result
            error_code = getattr(nav_result, 'error_code', None)
            error_msg = getattr(nav_result, 'error_msg', '')
            self.get_logger().warn(
                'Waypoint '
                f'{self.return_waypoint_index + 1} ended with status {status_label} ({status}). '
                f'error_code={error_code}, error_msg="{error_msg}". Continuing.'
            )

        self.return_waypoint_index += 1
        self._send_next_waypoint_goal()

    @staticmethod
    def _goal_status_to_text(status):
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCEEDED'
        if status == GoalStatus.STATUS_ABORTED:
            return 'ABORTED'
        if status == GoalStatus.STATUS_CANCELED:
            return 'CANCELED'
        if status == GoalStatus.STATUS_ACCEPTED:
            return 'ACCEPTED'
        if status == GoalStatus.STATUS_EXECUTING:
            return 'EXECUTING'
        if status == GoalStatus.STATUS_CANCELING:
            return 'CANCELING'
        return 'UNKNOWN'


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerReturnHome()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()