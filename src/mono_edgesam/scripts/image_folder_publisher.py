#!/usr/bin/env python3
# Copyright (c) 2025, D-Robotics.
#
# 从指定目录循环发布图片到ROS话题
# 用于将静态测试图片喂给 DOSOD / EdgeSAM 等算法节点

import os
import sys
import time
import glob

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageFolderPublisher(Node):
    def __init__(self):
        super().__init__('image_folder_publisher')

        self.declare_parameter('folder', '/home/sunrise/fast_ws/test_images')
        self.declare_parameter('topic', '/image_left_raw')
        self.declare_parameter('interval_ms', 200)  # 每帧间隔(ms)
        self.declare_parameter('loop', True)          # 是否循环发布
        self.declare_parameter('encoding', 'rgb8')    # 图像编码格式

        self.folder = self.get_parameter('folder').value
        self.topic = self.get_parameter('topic').value
        self.interval_ms = self.get_parameter('interval_ms').value
        self.loop = self.get_parameter('loop').value
        self.encoding = self.get_parameter('encoding').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.topic, 10)

        # 扫描图片文件
        exts = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
        self.image_files = []
        for ext in exts:
            self.image_files.extend(glob.glob(os.path.join(self.folder, ext)))
        self.image_files.sort()

        if not self.image_files:
            self.get_logger().error(f'No images found in {self.folder}')
            sys.exit(1)

        self.get_logger().info(
            f'Found {len(self.image_files)} images in {self.folder}')
        self.get_logger().info(
            f'Publishing to {self.topic} every {self.interval_ms}ms')

        self.idx = 0
        self.timer = self.create_timer(self.interval_ms / 1000.0, self.publish_next)

    def publish_next(self):
        if self.idx >= len(self.image_files):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info('All images published, shutting down.')
                rclpy.shutdown()
                return

        filepath = self.image_files[self.idx]
        self.get_logger().info(f'Publishing [{self.idx+1}/{len(self.image_files)}]: {filepath}')

        cv_img = cv2.imread(filepath, cv2.IMREAD_COLOR)
        if cv_img is None:
            self.get_logger().warn(f'Failed to read {filepath}')
            self.idx += 1
            return

        if self.encoding == 'rgb8':
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=self.encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

        self.idx += 1


def main():
    rclpy.init()
    node = ImageFolderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
