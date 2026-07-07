#!/usr/bin/env python3
"""Republish MVS camera frames as /rgb_img for web/FAST-LIVO consumers."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class RgbImgRepublisher(Node):
    def __init__(self):
        super().__init__('rgb_img_republisher')
        self.declare_parameter('input_topic', '/left_camera/image')
        self.declare_parameter('output_topic', '/rgb_img')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Image, output_topic, qos)
        self.create_subscription(Image, input_topic, self._on_image, qos)
        self.count = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'rgb_img_republisher: {input_topic} -> {output_topic}')

    def _on_image(self, msg):
        # Preserve the original header/encoding/data. This is a topic alias, not a conversion node.
        self.pub.publish(msg)
        self.count += 1

    def _report(self):
        self.get_logger().info(f'published /rgb_img frames: {self.count}')


def main(args=None):
    rclpy.init(args=args)
    node = RgbImgRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
