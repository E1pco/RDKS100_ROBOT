#!/usr/bin/env python3
"""Build a lightweight semantic voxel map from FAST-LIVO2 and AI perception."""

from __future__ import annotations

import json
import math
import os
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: Tuple[float, float, float, float]


@dataclass
class SemanticObservation:
    stamp_sec: float
    frame_id: str
    detections: List[Detection]
    mask_features: Optional[List[float]]
    mask_width: int
    mask_height: int


@dataclass
class Voxel:
    sum_x: float = 0.0
    sum_y: float = 0.0
    sum_z: float = 0.0
    observations: int = 0
    total_weight: float = 0.0
    class_weights: Dict[str, float] = None
    last_seen: float = 0.0

    def __post_init__(self) -> None:
        if self.class_weights is None:
            self.class_weights = defaultdict(float)

    def add(self, point: Vector3, label: str, weight: float, stamp_sec: float) -> None:
        self.sum_x += point[0]
        self.sum_y += point[1]
        self.sum_z += point[2]
        self.observations += 1
        self.total_weight += weight
        self.class_weights[label] += weight
        self.last_seen = stamp_sec

    @property
    def center(self) -> Vector3:
        if self.observations <= 0:
            return (0.0, 0.0, 0.0)
        inv = 1.0 / float(self.observations)
        return (self.sum_x * inv, self.sum_y * inv, self.sum_z * inv)

    @property
    def label_and_confidence(self) -> Tuple[str, float]:
        if not self.class_weights or self.total_weight <= 0.0:
            return ("unknown", 0.0)
        label, weight = max(self.class_weights.items(), key=lambda item: item[1])
        return (label, weight / self.total_weight)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def normalize_frame(frame_id: str) -> str:
    return frame_id[1:] if frame_id.startswith("/") else frame_id


def quat_to_matrix(x: float, y: float, z: float, w: float) -> Matrix3:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def mat_vec(matrix: Matrix3, vec: Vector3) -> Vector3:
    return (
        matrix[0][0] * vec[0] + matrix[0][1] * vec[1] + matrix[0][2] * vec[2],
        matrix[1][0] * vec[0] + matrix[1][1] * vec[1] + matrix[1][2] * vec[2],
        matrix[2][0] * vec[0] + matrix[2][1] * vec[1] + matrix[2][2] * vec[2],
    )


def mat_t_vec(matrix: Matrix3, vec: Vector3) -> Vector3:
    return (
        matrix[0][0] * vec[0] + matrix[1][0] * vec[1] + matrix[2][0] * vec[2],
        matrix[0][1] * vec[0] + matrix[1][1] * vec[1] + matrix[2][1] * vec[2],
        matrix[0][2] * vec[0] + matrix[1][2] * vec[1] + matrix[2][2] * vec[2],
    )


def vec_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


class SemanticMapBuilder(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_builder")

        self._declare_parameters()
        self._load_parameters()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_observation: Optional[SemanticObservation] = None
        self.voxels: Dict[Tuple[int, int, int], Voxel] = {}
        self.last_tf_warn_sec = 0.0
        self.last_sync_warn_sec = 0.0
        self.processed_frames = 0

        self.semantic_sub = self.create_subscription(
            PerceptionTargets,
            self.semantic_topic,
            self._semantic_callback,
            10,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.cloud_pub = self.create_publisher(PointCloud2, self.semantic_cloud_topic, 1)
        self.marker_pub = self.create_publisher(MarkerArray, self.semantic_marker_topic, 1)

        if self.publish_period > 0.0:
            self.publish_timer = self.create_timer(self.publish_period, self.publish_map)
        if self.save_interval > 0.0:
            self.save_timer = self.create_timer(self.save_interval, self.save_map)

        self.get_logger().info(
            "Semantic map builder: %s + %s -> %s"
            % (self.pointcloud_topic, self.semantic_topic, self.semantic_cloud_topic)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("pointcloud_topic", "/cloud_registered")
        self.declare_parameter("semantic_topic", "/perception/segmentation/edgesam")
        self.declare_parameter("semantic_cloud_topic", "/semantic_map/cloud")
        self.declare_parameter("semantic_marker_topic", "/semantic_map/markers")
        self.declare_parameter("global_frame", "camera_init")
        self.declare_parameter("lidar_frame", "aft_mapped")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("max_sync_age_sec", 0.5)

        self.declare_parameter("camera.width", 1224)
        self.declare_parameter("camera.height", 1024)
        self.declare_parameter("camera.fx", 1148.49)
        self.declare_parameter("camera.fy", 1137.96)
        self.declare_parameter("camera.cx", 721.322)
        self.declare_parameter("camera.cy", 477.389)
        self.declare_parameter(
            "camera.Rcl",
            [
                -0.091827,
                -0.994703,
                -0.046197,
                0.433230,
                0.001864,
                -0.901281,
                0.896593,
                -0.102776,
                0.430764,
            ],
        )
        self.declare_parameter("camera.Pcl", [-0.056560, -0.136283, -0.083827])

        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("max_depth", 12.0)
        self.declare_parameter("point_stride", 2)
        self.declare_parameter("max_points_per_frame", 50000)
        self.declare_parameter("min_detection_confidence", 0.25)
        self.declare_parameter("min_voxel_observations", 2)
        self.declare_parameter("min_voxel_confidence", 0.55)
        self.declare_parameter("max_voxels", 200000)
        self.declare_parameter("max_voxel_age_sec", 0.0)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("publish_max_voxels", 50000)
        self.declare_parameter("save_interval_sec", 10.0)
        self.declare_parameter("save_path", "/home/sunrise/fast_ws/src/fast_nav2_bringup/maps/semantic_map.json")
        self.declare_parameter("save_max_voxels", 100000)
        self.declare_parameter("target_classes", [])
        self.declare_parameter(
            "dynamic_classes",
            ["person", "bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat"],
        )
        self.declare_parameter("store_dynamic_classes", False)

    def _load_parameters(self) -> None:
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.semantic_cloud_topic = self.get_parameter("semantic_cloud_topic").value
        self.semantic_marker_topic = self.get_parameter("semantic_marker_topic").value
        self.global_frame = normalize_frame(self.get_parameter("global_frame").value)
        self.lidar_frame = normalize_frame(self.get_parameter("lidar_frame").value)
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.max_sync_age_sec = float(self.get_parameter("max_sync_age_sec").value)

        self.image_width = int(self.get_parameter("camera.width").value)
        self.image_height = int(self.get_parameter("camera.height").value)
        self.fx = float(self.get_parameter("camera.fx").value)
        self.fy = float(self.get_parameter("camera.fy").value)
        self.cx = float(self.get_parameter("camera.cx").value)
        self.cy = float(self.get_parameter("camera.cy").value)
        rcl = list(self.get_parameter("camera.Rcl").value)
        pcl = list(self.get_parameter("camera.Pcl").value)
        if len(rcl) != 9 or len(pcl) != 3:
            raise ValueError("camera.Rcl must have 9 values and camera.Pcl must have 3 values")
        self.rcl: Matrix3 = (
            (float(rcl[0]), float(rcl[1]), float(rcl[2])),
            (float(rcl[3]), float(rcl[4]), float(rcl[5])),
            (float(rcl[6]), float(rcl[7]), float(rcl[8])),
        )
        self.pcl: Vector3 = (float(pcl[0]), float(pcl[1]), float(pcl[2]))

        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))
        self.max_points_per_frame = max(1, int(self.get_parameter("max_points_per_frame").value))
        self.min_detection_confidence = float(self.get_parameter("min_detection_confidence").value)
        self.min_voxel_observations = max(1, int(self.get_parameter("min_voxel_observations").value))
        self.min_voxel_confidence = float(self.get_parameter("min_voxel_confidence").value)
        self.max_voxels = max(1, int(self.get_parameter("max_voxels").value))
        self.max_voxel_age_sec = float(self.get_parameter("max_voxel_age_sec").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.publish_period = 1.0 / publish_rate if publish_rate > 0.0 else 0.0
        self.publish_max_voxels = max(1, int(self.get_parameter("publish_max_voxels").value))
        self.save_interval = float(self.get_parameter("save_interval_sec").value)
        self.save_path = Path(os.path.expanduser(str(self.get_parameter("save_path").value)))
        self.save_max_voxels = max(1, int(self.get_parameter("save_max_voxels").value))
        self.target_classes = set(str(item) for item in self.get_parameter("target_classes").value)
        self.dynamic_classes = set(str(item) for item in self.get_parameter("dynamic_classes").value)
        self.store_dynamic_classes = bool(self.get_parameter("store_dynamic_classes").value)

    def _semantic_callback(self, msg: PerceptionTargets) -> None:
        observation = self._parse_semantic_observation(msg)
        if not observation.detections:
            return
        self.latest_observation = observation

    def _parse_semantic_observation(self, msg: PerceptionTargets) -> SemanticObservation:
        detections: List[Detection] = []
        mask_features: Optional[List[float]] = None
        mask_width = 0
        mask_height = 0

        for target in msg.targets:
            if target.type == "segmentation":
                if target.captures:
                    capture = target.captures[0]
                    mask_features = list(capture.features)
                    mask_width = int(capture.img.width)
                    mask_height = int(capture.img.height)
                continue

            for roi in target.rois:
                label = roi.type or target.type
                if not label:
                    continue
                confidence = float(roi.confidence)
                x1 = float(roi.rect.x_offset)
                y1 = float(roi.rect.y_offset)
                x2 = x1 + float(roi.rect.width)
                y2 = y1 + float(roi.rect.height)
                detections.append(Detection(label=label, confidence=confidence, bbox=(x1, y1, x2, y2)))

        if mask_features and len(mask_features) != mask_width * mask_height:
            self.get_logger().warn(
                "Ignoring malformed segmentation mask: %d values for %dx%d"
                % (len(mask_features), mask_width, mask_height)
            )
            mask_features = None
            mask_width = 0
            mask_height = 0

        return SemanticObservation(
            stamp_sec=stamp_to_sec(msg.header.stamp),
            frame_id=msg.header.frame_id,
            detections=detections,
            mask_features=mask_features,
            mask_width=mask_width,
            mask_height=mask_height,
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        observation = self.latest_observation
        if observation is None:
            return

        cloud_stamp = stamp_to_sec(msg.header.stamp)
        if cloud_stamp > 0.0 and observation.stamp_sec > 0.0:
            age = abs(cloud_stamp - observation.stamp_sec)
            if age > self.max_sync_age_sec:
                self._warn_sync_throttled(age)
                return

        try:
            transform_time = Time() if self.use_latest_tf else Time.from_msg(msg.header.stamp)
            global_from_lidar = self._lookup_transform(self.global_frame, self.lidar_frame, transform_time)
            cloud_frame = normalize_frame(msg.header.frame_id or self.global_frame)
            global_from_cloud = None
            if cloud_frame != self.global_frame:
                global_from_cloud = self._lookup_transform(self.global_frame, cloud_frame, transform_time)
        except TransformException as exc:
            self._warn_tf_throttled(str(exc))
            return

        points = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        if len(points) == 0:
            return

        selected_points = points[:: self.point_stride]
        if len(selected_points) > self.max_points_per_frame:
            selected_points = selected_points[: self.max_points_per_frame]

        added = 0
        for point in selected_points:
            if points.dtype.names:
                point_cloud = (float(point["x"]), float(point["y"]), float(point["z"]))
            else:
                point_cloud = (float(point[0]), float(point[1]), float(point[2]))

            point_global = self._apply_transform(global_from_cloud, point_cloud) if global_from_cloud else point_cloud
            point_lidar = self._apply_inverse_transform(global_from_lidar, point_global)
            point_camera = vec_add(mat_vec(self.rcl, point_lidar), self.pcl)
            pixel = self._project(point_camera)
            if pixel is None:
                continue

            detection = self._detection_for_pixel(observation, pixel[0], pixel[1])
            if detection is None:
                continue
            if detection.label in self.dynamic_classes and not self.store_dynamic_classes:
                continue

            self._add_voxel(point_global, detection, cloud_stamp or self.get_clock().now().seconds())
            added += 1

        self.processed_frames += 1
        if added:
            self._prune_if_needed(cloud_stamp or self.get_clock().now().seconds())

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp: Time) -> Tuple[Matrix3, Vector3]:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            stamp,
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
        translation = (float(t.x), float(t.y), float(t.z))
        return rotation, translation

    def _apply_transform(self, transform: Tuple[Matrix3, Vector3], point: Vector3) -> Vector3:
        rotation, translation = transform
        return vec_add(mat_vec(rotation, point), translation)

    def _apply_inverse_transform(self, transform: Tuple[Matrix3, Vector3], point: Vector3) -> Vector3:
        rotation, translation = transform
        return mat_t_vec(rotation, vec_sub(point, translation))

    def _project(self, point_camera: Vector3) -> Optional[Tuple[float, float]]:
        z = point_camera[2]
        if z <= self.min_depth or z >= self.max_depth:
            return None
        u = self.fx * point_camera[0] / z + self.cx
        v = self.fy * point_camera[1] / z + self.cy
        if u < 0.0 or v < 0.0 or u >= self.image_width or v >= self.image_height:
            return None
        return (u, v)

    def _detection_for_pixel(
        self,
        observation: SemanticObservation,
        u: float,
        v: float,
    ) -> Optional[Detection]:
        if observation.mask_features and observation.mask_width > 0 and observation.mask_height > 0:
            mx = int(u * float(observation.mask_width) / float(self.image_width))
            my = int(v * float(observation.mask_height) / float(self.image_height))
            if 0 <= mx < observation.mask_width and 0 <= my < observation.mask_height:
                label_index = int(round(observation.mask_features[my * observation.mask_width + mx]))
                if 1 <= label_index <= len(observation.detections):
                    detection = observation.detections[label_index - 1]
                    return detection if self._accept_detection(detection) else None

        best: Optional[Detection] = None
        for detection in observation.detections:
            if not self._accept_detection(detection):
                continue
            x1, y1, x2, y2 = detection.bbox
            if x1 <= u <= x2 and y1 <= v <= y2:
                if best is None or detection.confidence > best.confidence:
                    best = detection
        return best

    def _accept_detection(self, detection: Detection) -> bool:
        if detection.confidence < self.min_detection_confidence:
            return False
        return not self.target_classes or detection.label in self.target_classes

    def _add_voxel(self, point_global: Vector3, detection: Detection, stamp_sec: float) -> None:
        key = (
            math.floor(point_global[0] / self.voxel_size),
            math.floor(point_global[1] / self.voxel_size),
            math.floor(point_global[2] / self.voxel_size),
        )
        voxel = self.voxels.get(key)
        if voxel is None:
            voxel = Voxel()
            self.voxels[key] = voxel
        voxel.add(point_global, detection.label, max(detection.confidence, 0.01), stamp_sec)

    def _iter_valid_voxels(self, limit: int) -> Iterable[Tuple[Vector3, str, float, Voxel]]:
        emitted = 0
        for voxel in self.voxels.values():
            if voxel.observations < self.min_voxel_observations:
                continue
            label, confidence = voxel.label_and_confidence
            if confidence < self.min_voxel_confidence:
                continue
            yield voxel.center, label, confidence, voxel
            emitted += 1
            if emitted >= limit:
                return

    def publish_map(self) -> None:
        now = self.get_clock().now()
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.global_frame

        cloud_points = []
        marker_points = []
        marker_colors = []
        for center, label, confidence, _voxel in self._iter_valid_voxels(self.publish_max_voxels):
            color = self._color_for_label(label, confidence)
            rgba = self._rgba_uint32(color)
            cloud_points.append((center[0], center[1], center[2], rgba))
            marker_points.append(Point(x=center[0], y=center[1], z=center[2]))
            marker_colors.append(color)

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.cloud_pub.publish(pc2.create_cloud(header, fields, cloud_points))

        marker_array = MarkerArray()
        if not marker_points:
            delete_marker = Marker()
            delete_marker.header = header
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return

        marker = Marker()
        marker.header = header
        marker.ns = "semantic_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.points = marker_points
        marker.colors = marker_colors
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def save_map(self) -> None:
        self.save_path.parent.mkdir(parents=True, exist_ok=True)
        voxels = []
        for center, label, confidence, voxel in self._iter_valid_voxels(self.save_max_voxels):
            voxels.append(
                {
                    "x": center[0],
                    "y": center[1],
                    "z": center[2],
                    "label": label,
                    "confidence": confidence,
                    "observations": voxel.observations,
                    "last_seen": voxel.last_seen,
                }
            )
        data = {
            "frame_id": self.global_frame,
            "voxel_size": self.voxel_size,
            "processed_frames": self.processed_frames,
            "voxel_count": len(voxels),
            "voxels": voxels,
        }
        tmp_path = self.save_path.with_suffix(self.save_path.suffix + ".tmp")
        with tmp_path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp_path, self.save_path)

    def _prune_if_needed(self, now_sec: float) -> None:
        if self.max_voxel_age_sec > 0.0:
            stale = [
                key
                for key, voxel in self.voxels.items()
                if now_sec - voxel.last_seen > self.max_voxel_age_sec
            ]
            for key in stale:
                self.voxels.pop(key, None)

        if len(self.voxels) <= self.max_voxels:
            return
        remove_count = len(self.voxels) - self.max_voxels
        oldest = sorted(self.voxels.items(), key=lambda item: item[1].last_seen)[:remove_count]
        for key, _voxel in oldest:
            self.voxels.pop(key, None)

    def _color_for_label(self, label: str, confidence: float) -> ColorRGBA:
        palette = {
            "chair": (0.1, 0.6, 1.0),
            "couch": (0.3, 0.8, 0.5),
            "dining table": (1.0, 0.7, 0.1),
            "tv": (0.9, 0.2, 0.8),
            "laptop": (0.2, 0.9, 0.9),
            "bottle": (0.8, 0.9, 0.2),
            "cup": (1.0, 0.4, 0.2),
            "book": (0.5, 0.4, 1.0),
        }
        if label in palette:
            r, g, b = palette[label]
        else:
            seed = 0
            for char in label:
                seed = (seed * 131 + ord(char)) & 0xFFFFFF
            r = 0.2 + ((seed >> 0) & 0xFF) / 255.0 * 0.7
            g = 0.2 + ((seed >> 8) & 0xFF) / 255.0 * 0.7
            b = 0.2 + ((seed >> 16) & 0xFF) / 255.0 * 0.7
        alpha = max(0.25, min(1.0, confidence))
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))

    def _rgba_uint32(self, color: ColorRGBA) -> int:
        r = max(0, min(255, int(color.r * 255.0)))
        g = max(0, min(255, int(color.g * 255.0)))
        b = max(0, min(255, int(color.b * 255.0)))
        a = max(0, min(255, int(color.a * 255.0)))
        return (a << 24) | (r << 16) | (g << 8) | b

    def _warn_tf_throttled(self, message: str) -> None:
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.last_tf_warn_sec >= 5.0:
            self.last_tf_warn_sec = float(now)
            self.get_logger().warn("TF lookup failed: %s" % message)

    def _warn_sync_throttled(self, age: float) -> None:
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.last_sync_warn_sec >= 5.0:
            self.last_sync_warn_sec = float(now)
            self.get_logger().warn(
                "Semantic/cloud stamps differ by %.3fs; increase max_sync_age_sec if needed" % age
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
