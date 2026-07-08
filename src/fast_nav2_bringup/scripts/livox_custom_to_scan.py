#!/usr/bin/env python3
"""Convert Livox CustomMsg directly to a 2D LaserScan for Nav2."""

from __future__ import annotations

import math
from typing import List

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LivoxCustomToScan(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_scan")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("min_height", -0.20)
        self.declare_parameter("max_height", 0.40)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", 0.0087)
        self.declare_parameter("scan_time", 0.1)
        self.declare_parameter("range_min", 0.20)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("use_inf", True)
        self.declare_parameter("inf_epsilon", 1.0)
        self.declare_parameter("stamp_with_ros_time", True)
        self.declare_parameter("stamp_with_zero_time", False)
        self.declare_parameter("stamp_offset_sec", 0.05)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.scan_time = float(self.get_parameter("scan_time").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.use_inf = bool(self.get_parameter("use_inf").value)
        self.inf_epsilon = float(self.get_parameter("inf_epsilon").value)
        self.stamp_with_ros_time = bool(self.get_parameter("stamp_with_ros_time").value)
        self.stamp_with_zero_time = bool(self.get_parameter("stamp_with_zero_time").value)
        self.stamp_offset_sec = float(self.get_parameter("stamp_offset_sec").value)

        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")
        if self.angle_max <= self.angle_min:
            raise ValueError("angle_max must be greater than angle_min")

        self.bin_count = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))
        self.angle_max = self.angle_min + self.bin_count * self.angle_increment

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(LaserScan, self.scan_topic, qos)
        self.sub = self.create_subscription(CustomMsg, self.input_topic, self._callback, qos)
        self.get_logger().info(
            "Converting Livox CustomMsg %s -> LaserScan %s (%d bins, ros_stamp=%s, zero_stamp=%s, offset=%.3fs)"
            % (
                self.input_topic,
                self.scan_topic,
                self.bin_count,
                self.stamp_with_ros_time,
                self.stamp_with_zero_time,
                self.stamp_offset_sec,
            )
        )

    def _empty_ranges(self) -> List[float]:
        if self.use_inf:
            return [math.inf] * self.bin_count
        return [self.range_max + self.inf_epsilon] * self.bin_count

    def _callback(self, msg: CustomMsg) -> None:
        ranges = self._empty_ranges()

        for point in msg.points:
            z = float(point.z)
            if z < self.min_height or z > self.max_height:
                continue

            x = float(point.x)
            y = float(point.y)
            distance = math.hypot(x, y)
            if distance < self.range_min or distance > self.range_max:
                continue

            angle = math.atan2(y, x)
            if angle < self.angle_min or angle >= self.angle_max:
                continue

            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.bin_count and distance < ranges[index]:
                ranges[index] = distance

        scan = LaserScan()
        if self.stamp_with_zero_time:
            scan.header.stamp.sec = 0
            scan.header.stamp.nanosec = 0
        elif self.stamp_with_ros_time:
            scan_stamp = self.get_clock().now()
            if self.stamp_offset_sec:
                scan_stamp = scan_stamp + Duration(seconds=self.stamp_offset_sec)
            scan.header.stamp = scan_stamp.to_msg()
        else:
            scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_id or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxCustomToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
