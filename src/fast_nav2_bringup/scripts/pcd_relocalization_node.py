#!/usr/bin/env python3
"""Relocalize the live FAST-LIVO frame against a saved PCD map."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import TransformBroadcaster


Point2 = Tuple[float, float]
GridMap = Dict[Tuple[int, int], List[Point2]]


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def voxel_downsample_xy(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points[:, :2] / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def read_pcd_points(path: Path) -> np.ndarray:
    try:
        import open3d as o3d

        cloud = o3d.io.read_point_cloud(str(path))
        points = np.asarray(cloud.points, dtype=np.float64)
        if points.size > 0:
            return points
    except Exception:
        pass

    with path.open("rb") as f:
        raw = f.read()

    marker = b"DATA ascii"
    marker_index = raw.find(marker)
    if marker_index < 0:
        raise RuntimeError(
            f"{path} is not readable by open3d and is not an ASCII PCD file"
        )

    data_start = raw.find(b"\n", marker_index)
    if data_start < 0:
        raise RuntimeError(f"{path} has no PCD data payload")

    rows = []
    for line in raw[data_start + 1 :].decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        try:
            rows.append((float(parts[0]), float(parts[1]), float(parts[2])))
        except ValueError:
            continue
    return np.asarray(rows, dtype=np.float64)


def build_grid(points_xy: np.ndarray, cell_size: float) -> GridMap:
    grid: GridMap = {}
    if points_xy.size == 0:
        return grid
    cells = np.floor(points_xy / cell_size).astype(np.int64)
    for point, cell in zip(points_xy, cells):
        key = (int(cell[0]), int(cell[1]))
        grid.setdefault(key, []).append((float(point[0]), float(point[1])))
    return grid


def lookup_nearest(
    grid: GridMap,
    point: np.ndarray,
    cell_size: float,
    max_distance: float,
) -> Optional[Point2]:
    cell = np.floor(point[:2] / cell_size).astype(np.int64)
    radius = max(1, int(math.ceil(max_distance / cell_size)))
    best: Optional[Point2] = None
    best_d2 = max_distance * max_distance
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for candidate in grid.get((int(cell[0] + dx), int(cell[1] + dy)), []):
                diff_x = point[0] - candidate[0]
                diff_y = point[1] - candidate[1]
                d2 = diff_x * diff_x + diff_y * diff_y
                if d2 < best_d2:
                    best_d2 = d2
                    best = candidate
    return best


def estimate_rigid_2d(source: np.ndarray, target: np.ndarray) -> Tuple[float, np.ndarray]:
    src_mean = source.mean(axis=0)
    dst_mean = target.mean(axis=0)
    src_centered = source - src_mean
    dst_centered = target - dst_mean
    h = src_centered.T @ dst_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    yaw = math.atan2(r[1, 0], r[0, 0])
    translation = dst_mean - r @ src_mean
    return yaw, translation


class PcdRelocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("pcd_relocalization")

        self.declare_parameter(
            "map_pcd",
            "/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/Log/PCD/all_downsampled_points.pcd",
        )
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tracking_frame", "camera_init")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_pose_topic", "/relocalization/pose")
        self.declare_parameter("map_voxel_size", 0.20)
        self.declare_parameter("scan_voxel_size", 0.25)
        self.declare_parameter("icp_cell_size", 0.50)
        self.declare_parameter("max_correspondence_distance", 1.20)
        self.declare_parameter("min_correspondences", 80)
        self.declare_parameter("max_scan_points", 6000)
        self.declare_parameter("max_iterations", 8)
        self.declare_parameter("converged_translation", 0.03)
        self.declare_parameter("converged_yaw", 0.01)
        self.declare_parameter("run_period_sec", 1.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("z_min", -2.0)
        self.declare_parameter("z_max", 2.0)

        self.map_pcd = Path(str(self.get_parameter("map_pcd").value)).expanduser()
        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.tracking_frame = str(self.get_parameter("tracking_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.pose_topic = str(self.get_parameter("publish_pose_topic").value)
        self.map_voxel_size = float(self.get_parameter("map_voxel_size").value)
        self.scan_voxel_size = float(self.get_parameter("scan_voxel_size").value)
        self.icp_cell_size = float(self.get_parameter("icp_cell_size").value)
        self.max_correspondence_distance = float(
            self.get_parameter("max_correspondence_distance").value
        )
        self.min_correspondences = int(self.get_parameter("min_correspondences").value)
        self.max_scan_points = int(self.get_parameter("max_scan_points").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)
        self.converged_translation = float(
            self.get_parameter("converged_translation").value
        )
        self.converged_yaw = float(self.get_parameter("converged_yaw").value)
        self.run_period_sec = float(self.get_parameter("run_period_sec").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.translation = np.array(
            [
                float(self.get_parameter("initial_x").value),
                float(self.get_parameter("initial_y").value),
            ],
            dtype=np.float64,
        )

        self.map_points = self._load_map()
        self.map_grid = build_grid(self.map_points[:, :2], self.icp_cell_size)
        self.latest_cloud: Optional[PointCloud2] = None
        self.last_run_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initial_pose_callback,
            10,
        )

        self.get_logger().info(
            "PCD relocalization: %s (%d pts) + %s -> %s->%s"
            % (
                str(self.map_pcd),
                int(self.map_points.shape[0]),
                self.cloud_topic,
                self.map_frame,
                self.tracking_frame,
            )
        )

    def _load_map(self) -> np.ndarray:
        if not self.map_pcd.exists():
            raise RuntimeError(f"Map PCD does not exist: {self.map_pcd}")
        points = read_pcd_points(self.map_pcd)
        if points.size == 0:
            raise RuntimeError(f"Map PCD has no points: {self.map_pcd}")
        points = points[np.isfinite(points).all(axis=1)]
        points = points[(points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)]
        points = voxel_downsample_xy(points, self.map_voxel_size)
        if points.shape[0] < self.min_correspondences:
            raise RuntimeError(
                f"Map PCD has too few usable points after filtering: {points.shape[0]}"
            )
        return points

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.translation = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y],
            dtype=np.float64,
        )
        self.get_logger().info(
            "Initial relocalization pose set: x=%.3f y=%.3f yaw=%.3f"
            % (self.translation[0], self.translation[1], self.yaw)
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        now = self.get_clock().now()
        if (now - self.last_run_time).nanoseconds < self.run_period_sec * 1.0e9:
            self.latest_cloud = msg
            return
        self.last_run_time = now
        self.latest_cloud = msg
        try:
            scan = self._cloud_to_points(msg)
            if scan.shape[0] < self.min_correspondences:
                self.get_logger().warn(
                    "Skip relocalization: too few scan points (%d)" % scan.shape[0],
                    throttle_duration_sec=2.0,
                )
                return
            matched, rmse = self._align(scan)
            if matched < self.min_correspondences:
                self.get_logger().warn(
                    "Relocalization rejected: matches=%d rmse=%.3f" % (matched, rmse),
                    throttle_duration_sec=2.0,
                )
                return
            self._publish_pose(msg.header.stamp, rmse, matched)
        except Exception as exc:
            self.get_logger().warn(
                "Relocalization failed: %s" % exc,
                throttle_duration_sec=2.0,
            )

    def _cloud_to_points(self, msg: PointCloud2) -> np.ndarray:
        rows = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            z = float(point[2])
            if z < self.z_min or z > self.z_max:
                continue
            rows.append((float(point[0]), float(point[1]), z))
        if not rows:
            return np.empty((0, 3), dtype=np.float64)
        points = np.asarray(rows, dtype=np.float64)
        points = voxel_downsample_xy(points, self.scan_voxel_size)
        if self.max_scan_points > 0 and points.shape[0] > self.max_scan_points:
            step = int(math.ceil(points.shape[0] / float(self.max_scan_points)))
            points = points[::step]
        return points

    def _transform_xy(self, points: np.ndarray) -> np.ndarray:
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        return points[:, :2] @ rot.T + self.translation

    def _align(self, scan: np.ndarray) -> Tuple[int, float]:
        rmse = float("inf")
        matched_count = 0
        for _ in range(max(1, self.max_iterations)):
            source_xy = self._transform_xy(scan)
            source_matches = []
            target_matches = []
            for point in source_xy:
                nearest = lookup_nearest(
                    self.map_grid,
                    point,
                    self.icp_cell_size,
                    self.max_correspondence_distance,
                )
                if nearest is None:
                    continue
                source_matches.append(point)
                target_matches.append(nearest)
            matched_count = len(source_matches)
            if matched_count < self.min_correspondences:
                break

            source = np.asarray(source_matches, dtype=np.float64)
            target = np.asarray(target_matches, dtype=np.float64)
            delta_yaw, delta_t = estimate_rigid_2d(source, target)
            c = math.cos(delta_yaw)
            s = math.sin(delta_yaw)
            delta_r = np.array([[c, -s], [s, c]], dtype=np.float64)
            old_translation = self.translation.copy()
            self.yaw = normalize_angle(self.yaw + delta_yaw)
            self.translation = delta_r @ self.translation + delta_t

            residual = target - (source @ delta_r.T + delta_t)
            rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
            pose_delta = np.linalg.norm(self.translation - old_translation)
            if pose_delta < self.converged_translation and abs(delta_yaw) < self.converged_yaw:
                break
        return matched_count, rmse

    def _publish_pose(self, stamp, rmse: float, matched: int) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(self.translation[0])
        msg.pose.pose.position.y = float(self.translation[1])
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        variance = max(0.01, min(1.0, rmse * rmse))
        msg.pose.covariance[0] = variance
        msg.pose.covariance[7] = variance
        msg.pose.covariance[35] = max(0.01, min(0.5, rmse))
        self.pose_pub.publish(msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.tracking_frame
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        self.get_logger().info(
            "Relocalized x=%.3f y=%.3f yaw=%.3f matches=%d rmse=%.3f"
            % (self.translation[0], self.translation[1], self.yaw, matched, rmse),
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PcdRelocalizationNode()
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
