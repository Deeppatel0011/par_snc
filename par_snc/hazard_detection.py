#!/usr/bin/env python3
import math
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener


def distance_2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def median(values: List[float]) -> Optional[float]:
    if not values:
        return None
    vals = sorted(values)
    n = len(vals)
    if n % 2 == 1:
        return vals[n // 2]
    return 0.5 * (vals[n // 2 - 1] + vals[n // 2])


class HazardDetectionNode(Node):
    """
    Node 2: Hazard detection + placement on the map.

    Expected input topic from VM:
        /hazard_detections   std_msgs/String

    Expected message format:
        "<hazard_id>,<label>,<center_x_px>,<center_y_px>"

    Example:
        "12,Corrosive,378.0,242.0"

    This node runs on the ROBOT.
    It uses:
      - /hazard_detections  (from VM detector)
      - /scan               (robot laser)
      - TF map -> base_link (robot pose)

    It publishes:
      - /hazards    visualization_msgs/msg/Marker
      - /snc_status std_msgs/msg/String
    """

    def __init__(self):
        super().__init__('hazard_detection_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('laser_topic', '/scan'),
                ('detections_topic', '/hazard_detections'),
                ('hazards_topic', '/hazards'),
                ('status_topic', '/snc_status'),
                ('map_frame', 'map'),
                ('base_frame', 'base_link'),
                ('camera_hfov_deg', 69.0),
                ('camera_width_px', 640.0),
                ('marker_scale', 0.18),
                ('marker_z', 0.15),
                ('text_z', 0.40),
                ('marker_merge_distance', 0.45),
                ('marker_stability_hits', 3),
                ('publish_text_labels', True),
            ]
        )

        self.laser_topic = self.get_parameter('laser_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.hazards_topic = self.get_parameter('hazards_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_hfov_deg = float(self.get_parameter('camera_hfov_deg').value)
        self.camera_width_px = float(self.get_parameter('camera_width_px').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.marker_z = float(self.get_parameter('marker_z').value)
        self.text_z = float(self.get_parameter('text_z').value)
        self.marker_merge_distance = float(self.get_parameter('marker_merge_distance').value)
        self.marker_stability_hits = int(self.get_parameter('marker_stability_hits').value)
        self.publish_text_labels = bool(self.get_parameter('publish_text_labels').value)

        self.scan_msg: Optional[LaserScan] = None

        # Raw observations per hazard id
        self.marker_db: Dict[int, List[Tuple[float, float]]] = defaultdict(list)

        # Final confirmed hazards
        self.confirmed_markers: Dict[int, Tuple[float, float]] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, self.hazards_topic, 20)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(LaserScan, self.laser_topic, self.scan_cb, 20)
        self.create_subscription(String, self.detections_topic, self.detection_cb, 20)

        self.publish_status('Hazard detection node ready')

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def scan_cb(self, msg: LaserScan) -> None:
        
        self.scan_msg = msg

    def lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.25)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def parse_detection_string(self, text: str) -> Optional[Tuple[int, str, float, float]]:
        """
        Expected:
            12,Corrosive,378.0,242.0
        """
        try:
            raw_id, label, cx, cy = [part.strip() for part in text.split(',')]
            hazard_id = int(raw_id)
            center_x_px = float(cx)
            center_y_px = float(cy)
            return hazard_id, label, center_x_px, center_y_px
        except Exception as exc:
            self.get_logger().warn(f'Bad detection format: {text} ({exc})')
            return None

    def estimate_range_from_laser(self, center_x_px: float) -> Optional[float]:
        if self.scan_msg is None:
            return None

        hfov_rad = math.radians(self.camera_hfov_deg)
        angular_offset = ((center_x_px / self.camera_width_px) - 0.5) * hfov_rad

        scan = self.scan_msg
        index = int(round((angular_offset - scan.angle_min) / scan.angle_increment))
        indices = range(max(0, index - 3), min(len(scan.ranges), index + 4))

        valid = []
        for i in indices:
            r = scan.ranges[i]
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                valid.append(r)

        if not valid:
            return None

        return median(valid)

    def project_detection_to_map(
        self,
        center_x_px: float,
        distance: float,
        robot_pose: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        robot_x, robot_y, robot_yaw = robot_pose

        hfov_rad = math.radians(self.camera_hfov_deg)
        angular_offset = ((center_x_px / self.camera_width_px) - 0.5) * hfov_rad
        global_angle = robot_yaw + angular_offset

        hx = robot_x + distance * math.cos(global_angle)
        hy = robot_y + distance * math.sin(global_angle)
        return hx, hy

    def merge_or_store(self, hazard_id: int, point_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        # If already confirmed and close, keep existing
        if hazard_id in self.confirmed_markers:
            if distance_2d(self.confirmed_markers[hazard_id], point_xy) < self.marker_merge_distance:
                return self.confirmed_markers[hazard_id]

        self.marker_db[hazard_id].append(point_xy)

        if len(self.marker_db[hazard_id]) < self.marker_stability_hits:
            return None

        xs = [p[0] for p in self.marker_db[hazard_id]]
        ys = [p[1] for p in self.marker_db[hazard_id]]
        confirmed = (float(np.median(xs)), float(np.median(ys)))
        self.confirmed_markers[hazard_id] = confirmed
        return confirmed

    def publish_marker(self, hazard_id: int, label: str, xy: Tuple[float, float]) -> None:
        # Main hazard marker
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hazards'
        marker.id = int(hazard_id)  # important: actual hazard ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = xy[0]
        marker.pose.position.y = xy[1]
        marker.pose.position.z = self.marker_z
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

        # Optional text label
        if self.publish_text_labels:
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'hazard_labels'
            text_marker.id = int(hazard_id) + 5000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = xy[0]
            text_marker.pose.position.y = xy[1]
            text_marker.pose.position.z = self.text_z
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.14
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f'{hazard_id}:{label}'
            text_marker.lifetime.sec = 0
            self.marker_pub.publish(text_marker)

    def detection_cb(self, msg: String) -> None:
        self.get_logger().warn(f'Got the data from /detection_cb')
        if self.scan_msg is None:
            return

        parsed = self.parse_detection_string(msg.data)
        if parsed is None:
            return

        hazard_id, label, center_x_px, _center_y_px = parsed

        # Ignore unknown if needed
        if hazard_id == 0:
            return

        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            return

        distance = self.estimate_range_from_laser(center_x_px)
        if distance is None:
            self.get_logger().warn(f'Could not estimate hazard distance from laser for {hazard_id}')
            return

        hazard_xy = self.project_detection_to_map(center_x_px, distance, robot_pose)
        confirmed_xy = self.merge_or_store(hazard_id, hazard_xy)

        if confirmed_xy is not None:
            self.publish_marker(hazard_id, label, confirmed_xy)
            self.publish_status(
                f'Hazard confirmed: id={hazard_id} label={label} '
                f'at ({confirmed_xy[0]:.2f}, {confirmed_xy[1]:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = HazardDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()