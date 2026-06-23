#!/usr/bin/env python3
"""Check LiDAR, IMU, and camera timestamp alignment in a ROS 2 bag."""

from __future__ import annotations

import argparse
import bisect
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


DEFAULT_LIDAR_TOPIC = "/livox/lidar"
DEFAULT_IMAGE_TOPIC = "/left_camera/image"
DEFAULT_IMU_TOPIC = "/livox/imu"


@dataclass
class TopicStats:
    topic: str
    msg_type: str
    bag_times: List[int] = field(default_factory=list)
    header_times: List[int] = field(default_factory=list)
    header_minus_bag_times: List[int] = field(default_factory=list)
    missing_header: int = 0
    deserialize_errors: int = 0


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_sec(ns: int) -> float:
    return ns / 1_000_000_000.0


def fmt_ns_delta(ns: Optional[int]) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / 1_000_000.0:.3f} ms"


def fmt_abs_range(values_ns: Sequence[int]) -> str:
    if not values_ns:
        return "n/a"
    return f"{ns_to_sec(values_ns[0]):.9f} -> {ns_to_sec(values_ns[-1]):.9f}"


def detect_storage_id(bag_path: str) -> str:
    if os.path.isfile(bag_path) and bag_path.endswith(".mcap"):
        return "mcap"

    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if os.path.exists(metadata_path):
        with open(metadata_path, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("storage_identifier:"):
                    return stripped.split(":", 1)[1].strip()

    return "sqlite3"


def open_reader(bag_path: str) -> SequentialReader:
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=detect_storage_id(bag_path))
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def get_topic_types(reader: SequentialReader) -> Dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def get_header_stamp_ns(msg) -> Optional[int]:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return stamp_to_ns(stamp)


def count_reversals(times: Sequence[int]) -> int:
    return sum(1 for prev, cur in zip(times, times[1:]) if cur < prev)


def nearest_delta_ns(reference: int, candidates: Sequence[int]) -> Optional[int]:
    if not candidates:
        return None

    index = bisect.bisect_left(candidates, reference)
    best: Optional[int] = None
    if index < len(candidates):
        best = candidates[index] - reference
    if index > 0:
        left = candidates[index - 1] - reference
        if best is None or abs(left) < abs(best):
            best = left
    return best


def percentile_abs(values: Sequence[int], percent: float) -> Optional[int]:
    if not values:
        return None
    ordered = sorted(abs(value) for value in values)
    index = int(math.ceil((percent / 100.0) * len(ordered))) - 1
    index = max(0, min(index, len(ordered) - 1))
    return ordered[index]


def summarize_nearest(
    label: str,
    references: Sequence[int],
    candidates: Sequence[int],
    warn_ms: Optional[float],
) -> Tuple[str, bool]:
    deltas = [
        delta
        for delta in (nearest_delta_ns(reference, candidates) for reference in references)
        if delta is not None
    ]
    if not references:
        return f"{label}: no reference messages", False
    if not candidates:
        return f"{label}: no candidate messages", False
    if not deltas:
        return f"{label}: no comparable timestamps", False

    max_abs = max(abs(delta) for delta in deltas)
    ok = True
    mean_abs = sum(abs(delta) for delta in deltas) / len(deltas)
    signed_min = min(deltas)
    signed_max = max(deltas)
    status_text = "status=INFO"
    if warn_ms is not None:
        warn_ns = int(warn_ms * 1_000_000)
        ok = max_abs <= warn_ns
        status_text = f"threshold={warn_ms:.3f} ms, status={'OK' if ok else 'WARN'}"

    return (
        f"{label}: samples={len(deltas)}, "
        f"mean_abs={mean_abs / 1_000_000.0:.3f} ms, "
        f"p95_abs={fmt_ns_delta(percentile_abs(deltas, 95.0))}, "
        f"max_abs={fmt_ns_delta(max_abs)}, "
        f"signed_range={fmt_ns_delta(signed_min)}..{fmt_ns_delta(signed_max)}, "
        f"{status_text}",
        ok,
    )


def collect_stats(
    bag_path: str,
    topics: Sequence[str],
    max_messages: Optional[int],
    progress_every: int,
) -> Dict[str, TopicStats]:
    reader = open_reader(bag_path)
    topic_types = get_topic_types(reader)

    missing_topics = [topic for topic in topics if topic not in topic_types]
    if missing_topics:
        print("[ERROR] bag 中缺少话题:", ", ".join(missing_topics), file=sys.stderr)
        print("[ERROR] bag 中可用话题:", ", ".join(sorted(topic_types)), file=sys.stderr)
        sys.exit(2)

    msg_classes = {topic: get_message(topic_types[topic]) for topic in topics}
    stats = {
        topic: TopicStats(topic=topic, msg_type=topic_types[topic])
        for topic in topics
    }

    print("[Scan] bag:", bag_path, flush=True)
    for topic in topics:
        print(f"[Scan] topic: {topic} ({topic_types[topic]})", flush=True)

    target_read_count = 0
    start_time = time.monotonic()
    last_progress_time = start_time
    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic not in stats:
            continue

        target_read_count += 1
        item = stats[topic]
        item.bag_times.append(int(bag_time))
        try:
            msg = deserialize_message(data, msg_classes[topic])
            header_time = get_header_stamp_ns(msg)
            if header_time is None:
                item.missing_header += 1
            else:
                item.header_times.append(header_time)
                item.header_minus_bag_times.append(header_time - int(bag_time))
        except Exception as exc:  # noqa: BLE001 - report bad bag/message data and continue.
            item.deserialize_errors += 1
            if item.deserialize_errors <= 3:
                print(
                    f"[WARN] {topic} 反序列化失败: {exc}",
                    file=sys.stderr,
                )

        now = time.monotonic()
        if progress_every > 0 and (
            target_read_count % progress_every == 0 or now - last_progress_time >= 10.0
        ):
            counts = ", ".join(f"{name}={len(stats[name].bag_times)}" for name in topics)
            elapsed = now - start_time
            print(
                f"[Scan] read target messages={target_read_count}, elapsed={elapsed:.1f}s, {counts}",
                flush=True,
            )
            last_progress_time = now

        if max_messages is not None and target_read_count >= max_messages:
            break

    elapsed = time.monotonic() - start_time
    print(f"[Scan] finished target messages={target_read_count}, elapsed={elapsed:.1f}s", flush=True)
    return stats


def print_topic_summary(stats: Iterable[TopicStats]) -> None:
    print("\n[Topic summary]")
    for item in stats:
        count = len(item.bag_times)
        duration_ns = item.bag_times[-1] - item.bag_times[0] if count >= 2 else 0
        rate = (count - 1) / ns_to_sec(duration_ns) if duration_ns > 0 else 0.0
        header_count = len(item.header_times)
        offsets = item.header_minus_bag_times
        offset_text = "n/a"
        if offsets:
            mean_offset = sum(offsets) / len(offsets)
            offset_text = (
                f"mean={fmt_ns_delta(int(mean_offset))}, "
                f"min={fmt_ns_delta(min(offsets))}, max={fmt_ns_delta(max(offsets))}"
            )

        print(f"\n{item.topic}")
        print(f"  type: {item.msg_type}")
        print(f"  count: {count}, approx_rate_by_bag_time: {rate:.3f} Hz")
        print(f"  bag_time_range: {fmt_abs_range(item.bag_times)}")
        print(f"  header_stamp_range: {fmt_abs_range(item.header_times)}")
        print(f"  header_count: {header_count}, missing_header: {item.missing_header}")
        print(f"  bag_time_reversals: {count_reversals(item.bag_times)}")
        print(f"  header_stamp_reversals: {count_reversals(item.header_times)}")
        print(f"  header_minus_bag_time: {offset_text}")
        if item.deserialize_errors:
            print(f"  deserialize_errors: {item.deserialize_errors}")


def comparable_times(stats: Dict[str, TopicStats], topics: Sequence[str]) -> Tuple[str, Dict[str, List[int]]]:
    if all(stats[topic].header_times and len(stats[topic].header_times) == len(stats[topic].bag_times) for topic in topics):
        return "header.stamp", {topic: stats[topic].header_times for topic in topics}
    return "rosbag record time", {topic: stats[topic].bag_times for topic in topics}


def print_sync_summary(
    stats: Dict[str, TopicStats],
    lidar_topic: str,
    image_topic: str,
    imu_topic: str,
    image_lidar_warn_ms: float,
    image_imu_warn_ms: float,
) -> int:
    basis, times = comparable_times(stats, (lidar_topic, image_topic, imu_topic))
    print(f"\n[Sync summary] comparison_basis: {basis}")

    image_times = sorted(times[image_topic])
    lidar_times = sorted(times[lidar_topic])
    imu_times = sorted(times[imu_topic])

    checks = [
        summarize_nearest(
            "camera -> nearest lidar",
            image_times,
            lidar_times,
            image_lidar_warn_ms,
        ),
        summarize_nearest(
            "camera -> nearest imu",
            image_times,
            imu_times,
            image_imu_warn_ms,
        ),
        summarize_nearest(
            "lidar -> nearest camera (informational)",
            lidar_times,
            image_times,
            None,
        ),
    ]

    all_ok = True
    for line, ok in checks:
        print(line)
        all_ok = all_ok and ok

    print("\n[Interpretation]")
    print("- 硬同步优先看 header.stamp 的跨传感器差值，不优先看 rosbag 记录时间。")
    print("- camera -> nearest lidar 的 max_abs 越接近 0，说明相机和雷达触发时间越一致。")
    print("- camera -> nearest imu 用于确认每帧图像附近都有 IMU 数据，通常不要求完全同一时间戳。")
    print("- lidar -> nearest camera 是信息项；雷达频率高于相机时，不是每帧雷达都应匹配一帧相机。")
    print("- 如果 comparison_basis 显示为 rosbag record time，请先检查消息是否正确填充 header.stamp。")

    return 0 if all_ok else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate LiDAR, IMU, and camera timestamp alignment in a ROS 2 bag.",
    )
    parser.add_argument("bag", help="ROS 2 bag directory or storage file path")
    parser.add_argument("--lidar-topic", default=DEFAULT_LIDAR_TOPIC)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument(
        "--image-lidar-warn-ms",
        type=float,
        default=5.0,
        help="Warn when camera/lidar nearest timestamp max_abs exceeds this value.",
    )
    parser.add_argument(
        "--image-imu-warn-ms",
        type=float,
        default=10.0,
        help="Warn when camera/imu nearest timestamp max_abs exceeds this value.",
    )
    parser.add_argument(
        "--max-messages",
        "--max-test-count",
        dest="max_messages",
        type=int,
        default=None,
        help="Optional cap on total messages from the three target topics for quick checks.",
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=1000,
        help="Print scan progress every N target-topic messages. Use 0 to disable.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    topics = (args.lidar_topic, args.image_topic, args.imu_topic)
    stats = collect_stats(
        args.bag,
        topics,
        args.max_messages,
        args.progress_every,
    )
    print_topic_summary(stats[topic] for topic in topics)
    return print_sync_summary(
        stats,
        args.lidar_topic,
        args.image_topic,
        args.imu_topic,
        args.image_lidar_warn_ms,
        args.image_imu_warn_ms,
    )


if __name__ == "__main__":
    raise SystemExit(main())
