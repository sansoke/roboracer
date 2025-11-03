#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool  # (kept in case you extend)
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class DisparityExtender(Node):
    # constants (same as your ROS1 node unless noted)
    CAR_WIDTH = 0.45
    DIFFERENCE_THRESHOLD = 2.0
    STRAIGHTS_SPEED = 4.0
    CORNERS_SPEED = 3.0
    DRAG_SPEED = 4.0
    SAFETY_PERCENTAGE = 900.0  # extra safety margin (% of car_width/2)

    def __init__(self):
        super().__init__('disparity_extender_node')

        # Tunables (kept from your code)
        self.STEERING_SENSITIVITY = 3.0
        self.COEFFICIENT = 2.5
        self.EXP_COEFFICIENT = 0.02
        self.X_POWER = 1.8
        self.QUADRANT_FACTOR = 3.5

        # Parameters (so you can override in a launch file)
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('odom_topic', '/vesc/odom')
        self.declare_parameter('drive_topic', '/vesc/low_level/ackermann_cmd_mux/input/teleop')

        lidarscan_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        # State
        self.speed = 4.0
        self.radians_per_point = None  # updated on first scan

        # QoS for sensors: best-effort, keep last few samples
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions & publisher
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, qos_profile=sensor_qos
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.process_lidar, qos_profile=sensor_qos
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, qos_profile=QoSProfile(depth=10)
        )

        self.get_logger().info(
            f"DisparityExtender started.\n"
            f"  lidar_topic: {lidarscan_topic}\n"
            f"  odom_topic:  {odom_topic}\n"
            f"  drive_topic: {drive_topic}"
        )

    def odom_cb(self, data: Odometry):
        self.speed = data.twist.twist.linear.x

    def preprocess_lidar(self, ranges: np.ndarray) -> np.ndarray:
        # clip to reasonable max and remove rear sector
        ranges = np.clip(ranges, 0.0, 16.0)
        eighth = int(len(ranges) / self.QUADRANT_FACTOR)
        return np.asarray(ranges[eighth:-eighth])

    @staticmethod
    def get_differences(ranges: np.ndarray):
        # absolute difference between adjacent points
        diffs = np.zeros_like(ranges)
        diffs[1:] = np.abs(ranges[1:] - ranges[:-1])
        return diffs

    @staticmethod
    def get_disparities(differences: np.ndarray, threshold: float):
        return list(np.where(differences > threshold)[0])

    def get_num_points_to_cover(self, dist: float, width: float) -> int:
        # Use your original geometry, slightly conservative
        angle = 1.5 * np.arctan(width / (2.0 * max(dist, 1e-6)))
        # fall back if radians_per_point not known yet
        rpp = self.radians_per_point if self.radians_per_point else (np.deg2rad(360.0) / 1080.0)
        return int(np.ceil(angle / rpp))

    @staticmethod
    def cover_points(num_points: int, start_idx: int, cover_right: bool, ranges: np.ndarray):
        new_dist = ranges[start_idx]
        if cover_right:
            end = min(start_idx + 1 + num_points, len(ranges))
            mask = ranges[start_idx + 1:end] > new_dist
            ranges[start_idx + 1:end][mask] = new_dist
        else:
            begin = max(start_idx - num_points, 0)
            # going left
            seg = ranges[begin:start_idx]
            mask = seg > new_dist
            seg[mask] = new_dist
            ranges[begin:start_idx] = seg
        return ranges

    def extend_disparities(self, disparities, ranges: np.ndarray, car_width: float, extra_pct: float):
        # NOTE: original code used 0.155 m base; keeping as-is
        width_to_cover = 0.155 * (1.0 + extra_pct / 100.0)
        for index in disparities:
            first_idx = index - 1
            if first_idx < 0 or first_idx + 1 >= len(ranges):
                continue
            points = ranges[first_idx:first_idx + 2]
            close_rel = int(np.argmin(points))
            far_rel = 1 - close_rel
            close_idx = first_idx + close_rel
            far_idx = first_idx + far_rel

            close_dist = ranges[close_idx]
            num_points = self.get_num_points_to_cover(close_dist, width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points, close_idx, cover_right, ranges)
        return ranges

    def get_steering_angle(self, range_index: int, range_len: int):
        # use current radians_per_point
        rpp = self.radians_per_point if self.radians_per_point else (2.0 * np.pi) / max(range_len, 1)
        lidar_angle = (range_index - (range_len / 2.0)) * rpp
        steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90)) / self.STEERING_SENSITIVITY
        return float(steering_angle)

    def process_lidar(self, data: LaserScan):
        # Set radians_per_point from scan meta if available
        if data.angle_increment > 0.0:
            self.radians_per_point = float(data.angle_increment)
        else:
            # fallback: distribute 360 degrees over the sample count
            self.radians_per_point = (2.0 * np.pi) / max(len(data.ranges), 1)

        ranges = np.array(data.ranges, dtype=float)
        proc = self.preprocess_lidar(ranges)
        diffs = self.get_differences(proc)
        disparities = self.get_disparities(diffs, self.DIFFERENCE_THRESHOLD)
        proc = self.extend_disparities(disparities, proc, self.CAR_WIDTH, self.SAFETY_PERCENTAGE)

        # Choose the furthest point and compute steering
        best_idx = int(np.argmax(proc))
        steering_angle = self.get_steering_angle(best_idx, len(proc))

        # Robust version of your “x window” (center ±5 indices)
        center = len(proc) // 2
        left = max(center - 5, 0)
        right = min(center + 6, len(proc))
        x = float(np.max(proc[left:right]))

        speed = float(self.COEFFICIENT * math.exp(self.EXP_COEFFICIENT * (x ** self.X_POWER)))
        self.get_logger().info(f"x: {x:.3f}, speed: {speed:.3f}, steer(deg): {math.degrees(steering_angle):.1f}")

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = DisparityExtender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
