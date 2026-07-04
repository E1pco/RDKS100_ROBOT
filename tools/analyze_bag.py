#!/usr/bin/env python3
"""
分析rosbag中各话题的频率和时间同步情况。

用法:
    python3 tools/analyze_bag.py <bag_directory>

示例:
    python3 tools/analyze_bag.py ~/nas/bag/202606242
"""

import sqlite3
import sys
from pathlib import Path
from collections import defaultdict


def analyze_bag(bag_path):
    """分析rosbag的频率和同步情况。"""
    db_files = sorted(Path(bag_path).glob("*.db3"))
    if not db_files:
        print("❌ 未找到 .db3 文件!")
        return

    # 读取所有消息
    all_messages = []
    for db_file in db_files:
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute("SELECT id, name FROM topics")
        topics = {row[0]: row[1] for row in cursor.fetchall()}
        cursor.execute("SELECT topic_id, timestamp FROM messages ORDER BY timestamp")
        for row in cursor.fetchall():
            all_messages.append((topics[row[0]], row[1]))
        conn.close()

    # 按话题分组
    topic_msgs = defaultdict(list)
    for topic, ts in all_messages:
        topic_msgs[topic].append(ts)

    print("=" * 60)
    print(f"ROSbag 分析: {bag_path}")
    print("=" * 60)

    # 计算频率
    print("\n📊 话题频率:")
    print("-" * 60)
    for topic, timestamps in sorted(topic_msgs.items()):
        count = len(timestamps)
        duration = (timestamps[-1] - timestamps[0]) / 1e9
        freq = (count - 1) / duration if duration > 0 else 0
        print(f"  {topic:30s}: {count:6d} msgs, {freq:7.2f} Hz")

    # 同步分析
    cam_topic = "/left_camera/image"
    lidar_topic = "/livox/lidar"
    imu_topic = "/livox/imu"

    if cam_topic not in topic_msgs or lidar_topic not in topic_msgs:
        print("\n❌ 缺少相机或雷达数据")
        return

    cam_times = topic_msgs[cam_topic]
    lidar_times = topic_msgs[lidar_topic]
    imu_times = topic_msgs.get(imu_topic, [])

    # 采样分析
    print("\n⏱️  同步分析:")
    print("-" * 60)

    cam_lidar_diffs = []
    cam_imu_diffs = []
    lidar_imu_diffs = []

    sample_step = max(1, len(cam_times) // 20)

    for i in range(0, len(cam_times), sample_step):
        cam_t = cam_times[i]

        closest_lidar = min(lidar_times, key=lambda t: abs(t - cam_t))
        diff_cl = (closest_lidar - cam_t) / 1e6
        cam_lidar_diffs.append(diff_cl)

        if imu_times:
            closest_imu = min(imu_times, key=lambda t: abs(t - cam_t))
            diff_ci = (closest_imu - cam_t) / 1e6
            cam_imu_diffs.append(diff_ci)

            diff_li = (closest_imu - closest_lidar) / 1e6
            lidar_imu_diffs.append(diff_li)

    # 打印样本
    print("\n样本时间戳 (前5个):")
    for i in range(min(5, len(cam_lidar_diffs))):
        idx = i * sample_step
        print(f"  [{i+1}] Camera: {cam_times[idx]/1e9:.6f}s | "
              f"Cam-Lidar: {cam_lidar_diffs[i]:+.3f}ms | "
              f"Cam-IMU: {cam_imu_diffs[i]:+.3f}ms")

    # 统计信息
    print("\n📈 同步统计:")
    print("-" * 60)

    def print_stats(name, diffs):
        if not diffs:
            return
        avg = sum(diffs) / len(diffs)
        min_d = min(diffs)
        max_d = max(diffs)
        std = (sum((d - avg)**2 for d in diffs) / len(diffs)) ** 0.5
        print(f"  {name}:")
        print(f"    平均值: {avg:+.3f} ms")
        print(f"    最小值: {min_d:+.3f} ms")
        print(f"    最大值: {max_d:+.3f} ms")
        print(f"    标准差: {std:.3f} ms")

    print_stats("Camera-Lidar", cam_lidar_diffs)
    print_stats("Camera-IMU", cam_imu_diffs)
    print_stats("Lidar-IMU", lidar_imu_diffs)

    # 质量评估
    print("\n✅ 同步质量评估:")
    print("-" * 60)

    def assess(name, diffs, thresholds):
        if not diffs:
            return
        avg = abs(sum(diffs) / len(diffs))
        if avg < thresholds[0]:
            print(f"  {name}: ✅ 优秀 (< {thresholds[0]}ms)")
        elif avg < thresholds[1]:
            print(f"  {name}: ✅ 良好 (< {thresholds[1]}ms)")
        elif avg < thresholds[2]:
            print(f"  {name}: ⚠️  可接受 (< {thresholds[2]}ms)")
        else:
            print(f"  {name}: ❌ 较差 (> {thresholds[2]}ms)")

    assess("Camera-Lidar", cam_lidar_diffs, [1, 5, 10])
    assess("Camera-IMU", cam_imu_diffs, [1, 5, 10])
    assess("Lidar-IMU", lidar_imu_diffs, [5, 10, 20])

    # 时间基准检查
    print("\n🕐 时间基准检查:")
    print("-" * 60)
    if cam_times:
        print(f"  相机首个时间戳: {cam_times[0]/1e9:.6f} s")
    if lidar_times:
        print(f"  雷达首个时间戳: {lidar_times[0]/1e9:.6f} s")
    if imu_times:
        print(f"  IMU首个时间戳:  {imu_times[0]/1e9:.6f} s")

    if cam_times and lidar_times:
        diff = abs(cam_times[0] - lidar_times[0]) / 1e9
        if diff < 1.0:
            print(f"  ✅ 相机和雷达使用相同时间基准 (差值: {diff:.6f}s)")
        else:
            print(f"  ❌ 相机和雷达使用不同时间基准 (差值: {diff:.6f}s)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 analyze_bag.py <bag目录>")
        print("示例: python3 analyze_bag.py ~/nas/bag/202606242")
        sys.exit(1)

    analyze_bag(sys.argv[1])
