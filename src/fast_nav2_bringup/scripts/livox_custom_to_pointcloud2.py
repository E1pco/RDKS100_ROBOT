#!/usr/bin/env python3
"""Convert Livox CustomMsg point clouds to sensor_msgs/PointCloud2."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class LivoxCustomToPointCloud2(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_pointcloud2")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/points")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("transform_to_global_frame", False)
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("stamp_with_ros_time", False)
        self.declare_parameter("point_stride", 1)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._transform_to_global_frame = bool(self.get_parameter("transform_to_global_frame").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._stamp_with_ros_time = bool(self.get_parameter("stamp_with_ros_time").value)
        self._point_stride = max(1, int(self.get_parameter("point_stride").value))
        self._latest_odom = None

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
        self._odom_sub = None
        if self._transform_to_global_frame:
            self._odom_sub = self.create_subscription(
                Odometry,
                self._odom_topic,
                self._odom_callback,
                QoSProfile(depth=10),
            )
        self.get_logger().info(
            "Converting Livox CustomMsg %s -> PointCloud2 %s (global=%s, frame=%s, stride=%d)"
            % (
                input_topic,
                output_topic,
                self._transform_to_global_frame,
                self._frame_id or "<source>",
                self._point_stride,
            )
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg

    @staticmethod
    def _rotation_matrix(q):
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1e-12:
            return (
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
            )
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    def _callback(self, msg: CustomMsg) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg() if self._stamp_with_ros_time else msg.header.stamp
        header.frame_id = self._frame_id or msg.header.frame_id

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

        if self._transform_to_global_frame:
            odom = self._latest_odom
            if odom is None:
                return
            pos = odom.pose.pose.position
            rot = self._rotation_matrix(odom.pose.pose.orientation)

            def points():
                for index, point in enumerate(msg.points):
                    if index % self._point_stride != 0:
                        continue
                    x = float(point.x)
                    y = float(point.y)
                    z = float(point.z)
                    yield (
                        rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + pos.x,
                        rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + pos.y,
                        rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + pos.z,
                        float(point.reflectivity),
                    )
        else:
            def points():
                for index, point in enumerate(msg.points):
                    if index % self._point_stride != 0:
                        continue
                    yield (point.x, point.y, point.z, float(point.reflectivity))

        self._pub.publish(pc2.create_cloud(header, fields, points()))


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
