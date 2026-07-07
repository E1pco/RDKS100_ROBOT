#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from livox_ros_driver2.msg import CustomMsg


class LivoxRawCloudNode(Node):
    def __init__(self):
        super().__init__("livox_raw_cloud")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/points_raw")
        self.declare_parameter("max_points", 6000)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("frame_id", "livox_frame")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.max_points = max(1, int(self.get_parameter("max_points").value))
        self.publish_period = 1.0 / max(0.1, float(self.get_parameter("publish_rate").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.last_publish_time = 0.0

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 2)
        self.sub = self.create_subscription(CustomMsg, self.input_topic, self.on_livox, qos)
        self.get_logger().info(
            f"Republishing {self.input_topic} -> {self.output_topic}, "
            f"max_points={self.max_points}, rate={1.0 / self.publish_period:.1f}Hz"
        )

    def on_livox(self, msg: CustomMsg):
        now = time.monotonic()
        if now - self.last_publish_time < self.publish_period:
            return
        self.last_publish_time = now

        points = msg.points
        if not points:
            return

        stride = max(1, len(points) // self.max_points)
        selected = points[::stride]
        if len(selected) > self.max_points:
            selected = selected[:self.max_points]

        cloud = PointCloud2()
        cloud.header = msg.header
        if not cloud.header.frame_id:
            cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = len(selected)
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = bytearray(cloud.row_step)
        out_idx = 0
        for pt in selected:
            if not (math.isfinite(pt.x) and math.isfinite(pt.y) and math.isfinite(pt.z)):
                continue
            base = out_idx * cloud.point_step
            struct.pack_into("<ffff", data, base, float(pt.x), float(pt.y), float(pt.z), float(pt.reflectivity))
            out_idx += 1

        if out_idx == 0:
            return
        if out_idx != cloud.width:
            cloud.width = out_idx
            cloud.row_step = cloud.point_step * cloud.width
            data = data[:cloud.row_step]

        cloud.data = bytes(data)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = LivoxRawCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
