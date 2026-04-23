#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from par_snc.common import (
    BASE_FRAME,
    MAP_FRAME,
    PATH_EXPLORE_TOPIC,
    PATH_RETURN_TOPIC,
    STATUS_TOPIC,
    TRIGGER_HOME_TOPIC,
)


class PathNode(Node):
    def __init__(self):
        super().__init__('path_node')

        self.path_explore_pub = self.create_publisher(Path, PATH_EXPLORE_TOPIC, 10)
        self.path_return_pub = self.create_publisher(Path, PATH_RETURN_TOPIC, 10)

        self.return_mode = False
        self.last_mode = "explore"

        self.create_subscription(Empty, TRIGGER_HOME_TOPIC, self.home_callback, 10)
        self.create_subscription(String, STATUS_TOPIC, self.status_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.explore_points = []
        self.return_points = []

        self.min_point_distance = 0.12

        self.timer = self.create_timer(0.5, self.update_path)

        self.get_logger().info("Path Node Started")

    def home_callback(self, msg):
        if not self.return_mode:
            self.return_mode = True
            self.get_logger().info("Switching to RETURN PATH from /trigger_home")

    def status_callback(self, msg):
        status = msg.data.strip().lower()
        if "returning home" in status and not self.return_mode:
            self.return_mode = True
            self.get_logger().info("Switching to RETURN PATH from /snc_status")

    def get_robot_pose_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time()
            )
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def should_add_point(self, x, y, points_list):
        if not points_list:
            return True

        last_x, last_y = points_list[-1]
        dist = math.sqrt((x - last_x)**2 + (y - last_y)**2)
        return dist >= self.min_point_distance

    def build_path_msg(self, points):
        path = Path()
        path.header.frame_id = MAP_FRAME
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = MAP_FRAME
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def update_path(self):
        transform = self.get_robot_pose_in_map()
        if transform is None:
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        if not self.return_mode:
            if self.should_add_point(x, y, self.explore_points):
                self.explore_points.append((x, y))

            self.path_explore_pub.publish(self.build_path_msg(self.explore_points))

            if self.last_mode != "explore":
                self.get_logger().info("Publishing Explore Path")
                self.last_mode = "explore"

        else:
            if self.should_add_point(x, y, self.return_points):
                self.return_points.append((x, y))

            self.path_return_pub.publish(self.build_path_msg(self.return_points))

            if self.last_mode != "return":
                self.get_logger().info("Publishing Return Path")
                self.last_mode = "return"


def main():
    rclpy.init()
    node = PathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()