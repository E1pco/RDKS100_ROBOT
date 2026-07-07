#!/usr/bin/env python3
"""Compress MVS camera frames on S100 before sending them to the web host."""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class MvsJpegRepublisher(Node):
    def __init__(self):
        super().__init__('mvs_jpeg_republisher')
        self.declare_parameter('input_topic', '/left_camera/image')
        self.declare_parameter('output_topic', '/mvs/image_jpeg')
        self.declare_parameter('max_width', 640)
        self.declare_parameter('jpeg_quality', 45)
        self.declare_parameter('fps', 5.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.max_width = max(160, int(self.get_parameter('max_width').value))
        self.jpeg_quality = max(20, min(90, int(self.get_parameter('jpeg_quality').value)))
        fps = max(1.0, float(self.get_parameter('fps').value))
        self.min_period = 1.0 / fps
        self.last_pub_time = 0.0
        self.count = 0

        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(CompressedImage, self.output_topic, output_qos)
        self.create_subscription(Image, self.input_topic, self._on_image, input_qos)
        self._lock = threading.Lock()
        self._latest_msg = None
        self._latest_token = None
        self._encoded_token = None
        self.create_timer(max(0.01, self.min_period), self._publish_latest)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'mvs_jpeg_republisher: {self.input_topic} -> {self.output_topic}, '
            f'{fps:.1f} fps, max_width={self.max_width}, quality={self.jpeg_quality}'
        )

    def _ros_image_to_bgr(self, msg):
        encoding = (msg.encoding or '').lower()
        if encoding in ('rgb8', '8uc3'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step // 3, 3)[:, :msg.width, :]
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step // 3, 3)[:, :msg.width, :]
        if encoding in ('mono8', '8uc1'):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step)[:, :msg.width]
        if encoding in ('bgra8', 'rgba8', '8uc4'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.step // 4, 4)[:, :msg.width, :]
            code = cv2.COLOR_RGBA2BGR if encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(img, code)
        raise ValueError(f'unsupported encoding: {msg.encoding}')

    def _on_image(self, msg):
        token = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self._lock:
            self._latest_msg = msg
            self._latest_token = token

    def _publish_latest(self):
        now = time.monotonic()
        if now - self.last_pub_time < self.min_period:
            return

        with self._lock:
            msg = self._latest_msg
            token = self._latest_token
        if msg is None or token == self._encoded_token:
            return

        self.last_pub_time = now
        self._encoded_token = token

        try:
            img = self._ros_image_to_bgr(msg)
            if img.shape[1] > self.max_width:
                scale = self.max_width / float(img.shape[1])
                img = cv2.resize(
                    img,
                    (self.max_width, max(1, int(img.shape[0] * scale))),
                    interpolation=cv2.INTER_AREA,
                )
            ok, jpeg = cv2.imencode(
                '.jpg',
                img,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not ok:
                self.get_logger().warn('failed to encode MVS frame')
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = jpeg.tobytes()
            self.pub.publish(out)
            self.count += 1
        except Exception as exc:
            self.get_logger().warn(f'MVS JPEG conversion failed: {exc}')

    def _report(self):
        self.get_logger().info(f'published /mvs/image_jpeg frames: {self.count}')


def main(args=None):
    rclpy.init(args=args)
    node = MvsJpegRepublisher()
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
