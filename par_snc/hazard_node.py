#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from par_snc.common import HAZARD_TOPIC, MAP_FRAME, BASE_FRAME


HAZARD_IDS = {
    "unknown": 0,
    "explosive": 1,
    "flammable_gas": 2,
    "non_flammable_gas": 3,
    "dangerous_when_wet": 4,
    "flammable_solid": 5,
    "spontaneously_combustible": 6,
    "oxidizer": 7,
    "organic_peroxide": 8,
    "inhalation_hazard": 9,
    "poison": 10,
    "radioactive": 11,
    "corrosive": 12
}


OBJECT_NAME_TO_HAZARD_KEY = {
    "explosive": "explosive",
    "flammable_gas": "flammable_gas",
    "non_flammable_gas": "non_flammable_gas",
    "dangerous_when_wet": "dangerous_when_wet",
    "flammable_solid": "flammable_solid",
    "spontaneously_combustible": "spontaneously_combustible",
    "oxidizer": "oxidizer",
    "organic_peroxide": "organic_peroxide",
    "inhalation_hazard": "inhalation_hazard",
    "poison": "poison",
    "radioactive": "radioactive",
    "corrosive": "corrosive",
}


class HazardNode(Node):
    def __init__(self):
        super().__init__('hazard_node')

        self.hazard_pub = self.create_publisher(Marker, HAZARD_TOPIC, 10)

        # For now, manual testing:
        # ros2 topic pub /test_detected_object std_msgs/msg/String "{data: 'explosive'}" --once
        self.create_subscription(String, '/test_detected_object', self.test_detected_object_callback, 10)

        # Later you can replace/add a real find_object_2d subscription here.
        # Example:
        # self.create_subscription(String, '/camera_detected_object', self.detected_object_callback, 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.front_distance = None
        self.detected_hazards = {}

        self.min_valid_distance = 0.10
        self.max_valid_distance = 3.00

        self.get_logger().info("Hazard Node Started - Laser + TF")

    # =========================================================
    # Laser
    # =========================================================
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        if not ranges:
            return

        # front sector around 0 degrees
        front_sector = ranges[0:10] + ranges[-10:]

        valid = [
            r for r in front_sector
            if not math.isinf(r) and not math.isnan(r)
            and self.min_valid_distance <= r <= self.max_valid_distance
        ]

        if valid:
            self.front_distance = min(valid)
        else:
            self.front_distance = None

    # =========================================================
    # TF helpers
    # =========================================================
    def get_robot_pose_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            yaw = self.yaw_from_quaternion(qx, qy, qz, qw)

            return x, y, yaw

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed: map -> base_link")
            return None
        except Exception as e:
            self.get_logger().warn(f"Unexpected TF error: {e}")
            return None

    def yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # =========================================================
    # Marker publishing
    # =========================================================
    def publish_hazard_marker(self, hazard_key, x, y, z=0.20):
        marker = Marker()

        marker.header.frame_id = MAP_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "hazards"
        marker.id = HAZARD_IDS.get(hazard_key, 0)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.text = hazard_key

        self.hazard_pub.publish(marker)

        self.get_logger().info(
            f"Published hazard: key={hazard_key}, id={marker.id}, x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )

    # =========================================================
    # Detection handling
    # =========================================================
    def handle_detected_object(self, object_name):
        object_name = object_name.strip().lower()
        hazard_key = OBJECT_NAME_TO_HAZARD_KEY.get(object_name, "unknown")

        if hazard_key in self.detected_hazards:
            self.get_logger().info(f"Skipping duplicate hazard: {hazard_key}")
            return

        if self.front_distance is None:
            self.get_logger().warn("No valid front laser distance available")
            return

        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is None:
            self.get_logger().warn("Robot pose in map not available")
            return

        robot_x, robot_y, robot_yaw = robot_pose

        # Simple approximation:
        # assume marker is roughly in front of robot at current front laser distance
        hazard_x = robot_x + self.front_distance * math.cos(robot_yaw)
        hazard_y = robot_y + self.front_distance * math.sin(robot_yaw)

        self.detected_hazards[hazard_key] = {
            "x": hazard_x,
            "y": hazard_y,
            "z": 0.20
        }

        self.publish_hazard_marker(hazard_key, hazard_x, hazard_y, 0.20)

    def test_detected_object_callback(self, msg):
        self.get_logger().info(f"Manual test object received: {msg.data}")
        self.handle_detected_object(msg.data)


def main():
    rclpy.init()
    node = HazardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()