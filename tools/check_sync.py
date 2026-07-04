#!/usr/bin/env python3
"""
实时检查相机、雷达、IMU的时间同步情况。

用法:
    ros2 run fast_tools check_sync
    或
    python3 tools/check_sync.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu


class SyncChecker(Node):
    def __init__(self):
        super().__init__('sync_checker')
        self.cam = self.lidar = self.imu = None
        self.n = 0
        self.max_samples = 10

        self.create_subscription(Image, '/left_camera/image',
                                 lambda m: self.cb('cam', m), 10)
        self.create_subscription(PointCloud2, '/livox/lidar',
                                 lambda m: self.cb('lidar', m), 10)
        self.create_subscription(Imu, '/livox/imu',
                                 lambda m: self.cb('imu', m), 10)

        self.get_logger().info('Sync checker started, waiting for messages...')

    def cb(self, name, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        setattr(self, name, ts)

        if self.cam and self.lidar and self.imu:
            self.n += 1
            if self.n <= self.max_samples:
                cl = abs(self.cam - self.lidar) * 1000
                ci = abs(self.cam - self.imu) * 1000
                li = abs(self.lidar - self.imu) * 1000

                # 判断是否同一时间基准
                time_base = "SAME" if abs(self.cam - self.lidar) < 1.0 else "DIFFERENT"

                print(f"[{self.n:2d}] cam-lidar: {cl:8.3f}ms | cam-imu: {ci:8.3f}ms | "
                      f"lidar-imu: {li:8.3f}ms | time_base: {time_base}")

            if self.n == self.max_samples:
                print("\n✅ 测试完成")
                rclpy.shutdown()

            self.cam = self.lidar = self.imu = None


def main():
    rclpy.init()
    try:
        rclpy.spin(SyncChecker())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
