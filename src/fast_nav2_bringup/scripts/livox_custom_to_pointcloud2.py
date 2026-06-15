#!/usr/bin/env python3
"""Convert Livox CustomMsg point clouds to sensor_msgs/PointCloud2."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class LivoxCustomToPointCloud2(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_pointcloud2")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/points")
        self.declare_parameter("frame_id", "")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PointCloud2, output_topic, qos)
        self._sub = self.create_subscription(
            CustomMsg,
            input_topic,
            self._callback,
            qos,
        )
        self.get_logger().info(
            f"Converting Livox CustomMsg {input_topic} -> PointCloud2 {output_topic}"
        )

    def _callback(self, msg: CustomMsg) -> None:
        frame_id = self.get_parameter("frame_id").value or msg.header.frame_id
        header = msg.header
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        points = (
            (p.x, p.y, p.z, float(p.reflectivity))
            for p in msg.points
        )
        self._pub.publish(pc2.create_cloud(header, fields, points))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
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
