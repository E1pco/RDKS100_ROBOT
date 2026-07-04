#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

TOPIC = os.environ.get('RTSP_IMAGE_TOPIC', '/segmentation/image_jpeg')
URL = os.environ.get('RTSP_URL', 'rtsp://0.0.0.0:8554/camera')
FPS = os.environ.get('RTSP_FPS', '15')

class RtspPublisher(Node):
    def __init__(self):
        super().__init__('camera_rtsp_publisher')
        self.proc = None
        self.last_restart = 0.0
        self.count = 0
        self.sub = self.create_subscription(CompressedImage, TOPIC, self.on_frame, 10)
        self.start_ffmpeg()
        self.get_logger().info(f'RTSP publishing {TOPIC} -> {URL}')

    def start_ffmpeg(self):
        if self.proc and self.proc.poll() is None:
            return
        now = time.time()
        if now - self.last_restart < 2.0:
            return
        self.last_restart = now
        cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'warning',
            '-f', 'mjpeg', '-r', FPS, '-i', '-',
            '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
            '-pix_fmt', 'yuv420p', '-f', 'rtsp', '-listen', '1', URL,
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def on_frame(self, msg):
        self.start_ffmpeg()
        if not self.proc or self.proc.poll() is not None or not self.proc.stdin:
            return
        try:
            self.proc.stdin.write(bytes(msg.data))
            self.proc.stdin.flush()
            self.count += 1
            if self.count % 150 == 0:
                self.get_logger().info(f'pushed {self.count} frames to RTSP')
        except (BrokenPipeError, OSError):
            try:
                self.proc.kill()
            except Exception:
                pass
            self.proc = None

    def destroy_node(self):
        if self.proc:
            try:
                self.proc.send_signal(signal.SIGTERM)
            except Exception:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    node = RtspPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
