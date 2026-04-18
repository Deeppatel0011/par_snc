#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from par_snc.common import STATUS_TOPIC


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # -----------------------------
        # Publishers
        # -----------------------------
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Empty, '/trigger_start', self.start_callback, 10)
        self.create_subscription(Empty, '/trigger_home', self.home_callback, 10)
        self.create_subscription(Empty, '/trigger_teleop', self.teleop_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # -----------------------------
        # Nav2 action client
        # -----------------------------
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # State
        # -----------------------------
        self.state = "waiting_for_start"
        self.map_msg = None
        self.home_pose = None
        self.start_received = False
        self.exploration_complete = False

        # Goal / frontier handling
        self.current_goal_handle = None
        self.current_goal_future = None
        self.current_result_future = None
        self.current_goal_xy = None
        self.goal_start_time = None
        self.goal_timeout_sec = 55.0

        # Frontier memory
        self.failed_frontiers = []       # list of (x, y, time_sec, fail_count)
        self.visited_frontiers = []      # list of (x, y, time_sec)

        self.last_frontier_retry_time = 0.0
        self.frontier_retry_wait_sec = 2.0

        # Recovery
        self.recovery_mode = False
        self.recovery_end_time = None
        self.recovery_phase = "none"
        self.recovery_turn_direction = 1.0

        # Laser distances
        self.front_distance = None
        self.front_left_distance = None
        self.front_right_distance = None
        self.left_distance = None
        self.right_distance = None

        # -----------------------------
        # Tuning - relaxed for maze
        # -----------------------------
        self.safe_front_distance = 0.70
        self.emergency_front_distance = 0.32
        self.side_safe_distance = 0.30

        self.recovery_back_speed = -0.07
        self.recovery_turn_speed = 0.7

        # Frontier settings
        self.min_frontier_cluster_size = 5
        self.frontier_blacklist_radius = 0.28
        self.frontier_visited_radius = 0.35
        self.frontier_goal_tolerance = 0.45
        self.min_frontier_distance_from_robot = 0.35
        self.max_fail_count_per_region = 3

        # Timers
        self.control_timer = self.create_timer(0.2, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("Navigation Node Started - Improved Frontier Exploration")

    # =========================================================
    # Trigger callbacks
    # =========================================================
    def start_callback(self, msg):
        if self.state == "teleoperation":
            return

        self.start_received = True
        self.exploration_complete = False

        pose = self.get_robot_pose()
        if pose is not None and self.home_pose is None:
            self.home_pose = self.make_pose_stamped(pose[0], pose[1], pose[2], frame_id="map")
            self.get_logger().info(f"Saved home pose at x={pose[0]:.2f}, y={pose[1]:.2f}")

        self.state = "exploring_frontiers"
        self.get_logger().info("START TRIGGER RECEIVED -> exploring_frontiers")

    def home_callback(self, msg):
        self.cancel_current_goal()
        self.stop_robot()
        self.recovery_mode = False

        if self.home_pose is None:
            pose = self.get_robot_pose()
            if pose is not None:
                self.home_pose = self.make_pose_stamped(pose[0], pose[1], pose[2], frame_id="map")

        if self.home_pose is not None:
            sent = self.send_pose_goal(self.home_pose)
            if sent:
                self.state = "returning_home"
                self.get_logger().info("RETURN HOME TRIGGER RECEIVED -> returning_home")
            else:
                self.state = "waiting_for_start"
                self.get_logger().warn("Could not send home goal")
        else:
            self.state = "waiting_for_start"
            self.get_logger().warn("RETURN HOME TRIGGER RECEIVED but no home pose available")

    def teleop_callback(self, msg):
        self.cancel_current_goal()
        self.stop_robot()
        self.recovery_mode = False
        self.state = "teleoperation"
        self.get_logger().info("TELEOP TRIGGER RECEIVED -> teleoperation")

    # =========================================================
    # Sensor callbacks
    # =========================================================
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        if not ranges:
            return

        def clean_min(values):
            valid = [v for v in values if not math.isinf(v) and not math.isnan(v) and v > 0.0]
            if not valid:
                return None
            return min(valid)

        n = len(ranges)

        front_sector = ranges[0:20] + ranges[-20:]
        front_left_sector = ranges[20:55]
        left_sector = ranges[n // 4 - 15:n // 4 + 15]
        front_right_sector = ranges[-55:-20]
        right_sector = ranges[3 * n // 4 - 15:3 * n // 4 + 15]

        self.front_distance = clean_min(front_sector)
        self.front_left_distance = clean_min(front_left_sector)
        self.front_right_distance = clean_min(front_right_sector)
        self.left_distance = clean_min(left_sector)
        self.right_distance = clean_min(right_sector)

    def map_callback(self, msg):
        self.map_msg = msg

    # =========================================================
    # Status
    # =========================================================
    def get_status_text(self):
        if self.state == "teleoperation":
            return "Teleoperation Mode"
        if self.state == "returning_home":
            return "Returning Home"
        if self.state == "finished":
            return "Finished"
        if self.state == "waiting_for_start":
            return "Waiting for Start"
        if self.state == "exploring_frontiers":
            if self.recovery_mode:
                return "Exploring - Recovery"
            return "Exploring Frontiers"
        if self.state == "navigating_to_frontier":
            if self.recovery_mode:
                return "Navigating - Recovery"
            return "Navigating To Frontier"
        return "Navigation Active"

    def publish_status(self):
        msg = String()
        msg.data = self.get_status_text()
        self.status_pub.publish(msg)

    # =========================================================
    # Utility: cmd_vel
    # =========================================================
    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    # =========================================================
    # Utility: time
    # =========================================================
    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # =========================================================
    # Utility: TF pose
    # =========================================================
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = self.yaw_from_quaternion(qx, qy, qz, qw)

            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        except Exception:
            return None

    def yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_from_yaw(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def make_pose_stamped(self, x, y, yaw, frame_id="map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # =========================================================
    # Nav2 helpers
    # =========================================================
    def send_pose_goal(self, pose_stamped):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateToPose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.current_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.current_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_start_time = self.now_sec()
        self.current_goal_xy = (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y
        )

        self.get_logger().info(
            f"Sent goal to x={self.current_goal_xy[0]:.2f}, y={self.current_goal_xy[1]:.2f}"
        )
        return True

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal request failed: {e}")
            self.current_goal_handle = None
            self.current_result_future = None
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.current_goal_handle = None
            self.current_result_future = None
            if self.current_goal_xy is not None:
                self.add_failed_frontier(self.current_goal_xy[0], self.current_goal_xy[1])
            return

        self.current_goal_handle = goal_handle
        self.current_result_future = goal_handle.get_result_async()
        self.current_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal result error: {e}")
            if self.current_goal_xy is not None:
                self.add_failed_frontier(self.current_goal_xy[0], self.current_goal_xy[1])
            self.current_goal_handle = None
            self.current_result_future = None
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 goal succeeded")
            if self.current_goal_xy is not None:
                self.add_visited_frontier(self.current_goal_xy[0], self.current_goal_xy[1])

            if self.state == "returning_home":
                self.state = "finished"
                self.stop_robot()
            else:
                self.state = "exploring_frontiers"

        else:
            self.get_logger().warn(f"Nav2 goal failed with status={status}")
            if self.current_goal_xy is not None:
                self.add_failed_frontier(self.current_goal_xy[0], self.current_goal_xy[1])

            if self.state == "returning_home":
                self.state = "finished"
            else:
                self.state = "exploring_frontiers"

        self.current_goal_handle = None
        self.current_result_future = None
        self.current_goal_xy = None
        self.goal_start_time = None

    def cancel_current_goal(self):
        try:
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
        except Exception:
            pass

        self.current_goal_handle = None
        self.current_result_future = None
        self.current_goal_future = None
        self.current_goal_xy = None
        self.goal_start_time = None

    # =========================================================
    # Recovery
    # =========================================================
    def start_recovery(self, turn_left=True):
        self.cancel_current_goal()

        self.recovery_mode = True
        self.recovery_phase = "backing"
        self.recovery_turn_direction = 1.0 if turn_left else -1.0
        self.recovery_end_time = self.now_sec() + 0.6

        self.get_logger().warn(f"Recovery started, turn_left={turn_left}")

    def run_recovery(self):
        if not self.recovery_mode:
            return False

        now = self.now_sec()

        if self.recovery_phase == "backing":
            if now < self.recovery_end_time:
                self.publish_cmd(self.recovery_back_speed, 0.0)
                return True
            self.recovery_phase = "turning"
            self.recovery_end_time = now + 0.9
            return True

        if self.recovery_phase == "turning":
            if now < self.recovery_end_time:
                self.publish_cmd(0.0, self.recovery_turn_speed * self.recovery_turn_direction)
                return True

            self.recovery_mode = False
            self.recovery_phase = "none"
            self.stop_robot()

            if self.state not in ["returning_home", "teleoperation"]:
                self.state = "exploring_frontiers"
            return False

        return False

    # =========================================================
    # Frontier memory
    # =========================================================
    def add_failed_frontier(self, x, y):
        now = self.now_sec()

        for i, (fx, fy, ft, count) in enumerate(self.failed_frontiers):
            if math.hypot(x - fx, y - fy) < self.frontier_blacklist_radius:
                self.failed_frontiers[i] = (fx, fy, now, count + 1)
                return

        self.failed_frontiers.append((x, y, now, 1))

    def add_visited_frontier(self, x, y):
        self.visited_frontiers.append((x, y, self.now_sec()))

    def is_near_failed_frontier(self, x, y):
        for fx, fy, _, count in self.failed_frontiers:
            if math.hypot(x - fx, y - fy) < self.frontier_blacklist_radius:
                if count >= self.max_fail_count_per_region:
                    return True
        return False

    def is_near_visited_frontier(self, x, y):
        for vx, vy, _ in self.visited_frontiers:
            if math.hypot(x - vx, y - vy) < self.frontier_visited_radius:
                return True
        return False

    # =========================================================
    # Map / frontier helpers
    # =========================================================
    def map_index(self, mx, my, width):
        return my * width + mx

    def map_to_world(self, mx, my):
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        resolution = self.map_msg.info.resolution

        wx = origin_x + (mx + 0.5) * resolution
        wy = origin_y + (my + 0.5) * resolution
        return wx, wy

    def is_inside_map(self, mx, my, width, height):
        return 0 <= mx < width and 0 <= my < height

    def is_frontier_cell(self, mx, my, grid, width, height):
        idx = self.map_index(mx, my, width)
        if grid[idx] != 0:
            return False

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx = mx + dx
            ny = my + dy
            if self.is_inside_map(nx, ny, width, height):
                nidx = self.map_index(nx, ny, width)
                if grid[nidx] == -1:
                    return True
        return False

    def is_safe_goal_cell(self, mx, my, grid, width, height, inflation=1):
        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                nx = mx + dx
                ny = my + dy
                if not self.is_inside_map(nx, ny, width, height):
                    return False
                nidx = self.map_index(nx, ny, width)
                if grid[nidx] > 50:
                    return False
        return True

    def get_frontier_goal(self):
        if self.map_msg is None:
            return None

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None

        rx, ry, ryaw = robot_pose

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid = list(self.map_msg.data)

        frontier_cells = set()
        for my in range(1, height - 1):
            for mx in range(1, width - 1):
                if self.is_frontier_cell(mx, my, grid, width, height):
                    frontier_cells.add((mx, my))

        if not frontier_cells:
            return None

        clusters = []
        visited = set()

        for cell in frontier_cells:
            if cell in visited:
                continue

            q = deque([cell])
            visited.add(cell)
            cluster = []

            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))

                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        nx = cx + dx
                        ny = cy + dy
                        ncell = (nx, ny)
                        if ncell in frontier_cells and ncell not in visited:
                            visited.add(ncell)
                            q.append(ncell)

            if len(cluster) >= self.min_frontier_cluster_size:
                clusters.append(cluster)

        if not clusters:
            return None

        best_goal = None
        best_score = -1e9

        for cluster in clusters:
            avg_mx = sum(c[0] for c in cluster) / len(cluster)
            avg_my = sum(c[1] for c in cluster) / len(cluster)

            candidate_cells = sorted(
                cluster,
                key=lambda c: (c[0] - avg_mx) ** 2 + (c[1] - avg_my) ** 2
            )

            chosen = None
            for cmx, cmy in candidate_cells:
                if self.is_safe_goal_cell(cmx, cmy, grid, width, height, inflation=1):
                    chosen = (cmx, cmy)
                    break

            if chosen is None:
                continue

            wx, wy = self.map_to_world(chosen[0], chosen[1])
            dist_to_robot = math.hypot(wx - rx, wy - ry)

            if dist_to_robot < self.min_frontier_distance_from_robot:
                continue

            if self.is_near_failed_frontier(wx, wy):
                continue

            if self.is_near_visited_frontier(wx, wy):
                continue

            # Better scoring:
            # prefer larger clusters and meaningful exploration distance
            # but avoid extremely far useless targets
            cluster_size = len(cluster)
            score = (2.5 * cluster_size) + (1.2 * dist_to_robot)

            if dist_to_robot > 5.5:
                score -= 3.0

            if score > best_score:
                best_score = score
                best_goal = (wx, wy, ryaw)

        return best_goal

    # =========================================================
    # Main control loop
    # =========================================================
    def control_loop(self):
        if self.state in ["waiting_for_start", "teleoperation", "finished"]:
            return

        if self.front_distance is not None and self.front_distance < self.emergency_front_distance:
            if not self.recovery_mode:
                turn_left = True
                if self.left_distance is not None and self.right_distance is not None:
                    turn_left = self.left_distance > self.right_distance
                self.start_recovery(turn_left=turn_left)
            self.run_recovery()
            return

        if self.recovery_mode:
            self.run_recovery()
            return

        if self.state == "returning_home":
            if self.goal_start_time is not None:
                if (self.now_sec() - self.goal_start_time) > self.goal_timeout_sec:
                    self.get_logger().warn("Home goal timeout")
                    self.cancel_current_goal()
                    self.state = "finished"
                    self.stop_robot()
            return

        if self.map_msg is None:
            self.stop_robot()
            return

        if self.state == "navigating_to_frontier":
            if self.goal_start_time is not None:
                if (self.now_sec() - self.goal_start_time) > self.goal_timeout_sec:
                    self.get_logger().warn("Frontier goal timeout")
                    if self.current_goal_xy is not None:
                        self.add_failed_frontier(self.current_goal_xy[0], self.current_goal_xy[1])
                    self.cancel_current_goal()
                    self.state = "exploring_frontiers"
                    return

            # only recover if really squeezed
            side_too_close = False
            if self.front_left_distance is not None and self.front_left_distance < self.side_safe_distance:
                side_too_close = True
            if self.front_right_distance is not None and self.front_right_distance < self.side_safe_distance:
                side_too_close = True

            if side_too_close and self.front_distance is not None and self.front_distance < self.safe_front_distance:
                turn_left = True
                if self.left_distance is not None and self.right_distance is not None:
                    turn_left = self.left_distance > self.right_distance
                self.start_recovery(turn_left=turn_left)
                return

            return

        if self.state == "exploring_frontiers":
            now = self.now_sec()
            if now - self.last_frontier_retry_time < self.frontier_retry_wait_sec:
                return

            self.last_frontier_retry_time = now

            frontier_goal = self.get_frontier_goal()

            if frontier_goal is None:
                self.get_logger().info("No more valid frontiers found -> exploration complete")
                self.exploration_complete = True

                if self.home_pose is not None:
                    sent = self.send_pose_goal(self.home_pose)
                    if sent:
                        self.state = "returning_home"
                    else:
                        self.state = "finished"
                        self.stop_robot()
                else:
                    self.state = "finished"
                    self.stop_robot()
                return

            fx, fy, fyaw = frontier_goal
            pose = self.make_pose_stamped(fx, fy, fyaw, frame_id="map")

            sent = self.send_pose_goal(pose)
            if sent:
                self.state = "navigating_to_frontier"
            else:
                self.add_failed_frontier(fx, fy)
                self.state = "exploring_frontiers"

    # =========================================================
    # Shutdown
    # =========================================================
    def destroy_node(self):
        try:
            self.cancel_current_goal()
            self.stop_robot()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()