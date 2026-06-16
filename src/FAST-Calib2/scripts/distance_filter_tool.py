#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：
1) 自动检测 rosbag 中雷达点云类型：
   - sensor_msgs/PointCloud2  (如 /hesai/pandar)
   - livox_ros_driver/CustomMsg (如 /livox/lidar)
2) 按各自的解析方式把点云导出成一个带 intensity 的 PCD 文件 (x y z intensity, ASCII)
3) 使用 Open3D 对该 PCD 进行交互选点（至少 4 个点），并根据 4 个点计算包围范围，
   保存为同名 .txt 文件。

依赖：
    - rosbag2_py
    - sensor_msgs_py.point_cloud2
    - open3d, numpy

用法示例：
    python FAST-Calib-tool.py
    python FAST-Calib-tool.py /path/to/data.bag /path/to/output_dir
"""

import os
import sys
import math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

try:
    import open3d as o3d
except ImportError:
    o3d = None

try:
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    get_message = None

PREVIEW_VOXEL_SIZE = float(os.environ.get("FAST_CALIB_PREVIEW_VOXEL", "0.08"))
SAVE_FULL_PCD = os.environ.get("FAST_CALIB_SAVE_FULL_PCD", "0").lower() in ("1", "true", "yes", "on")

# ===================== 通用：保存 PCD =====================

def write_pcd_header(file_obj, point_count):
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {point_count}
HEIGHT 1
POINTS {point_count}
DATA ascii
"""
    file_obj.write(header)


def save_pcd_with_intensity(point_rows, output_path):
    """
    保存点云为带 intensity 字段的 PCD 文件 (ASCII 格式)
    point_rows: iterable of (x, y, z, intensity)
    """
    point_count = len(point_rows)
    with open(output_path, 'w') as f:
        write_pcd_header(f, point_count)
        for x, y, z, inten in point_rows:
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] 保存带 intensity 字段的点云到: {output_path}")


class VoxelPreviewBuffer:
    """用体素哈希保留代表点，避免把整包点云加载进内存和 Open3D。"""

    def __init__(self, voxel_size):
        self.voxel_size = voxel_size
        self.voxels = {}
        self.total_points = 0

    def add(self, x, y, z, intensity):
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return

        self.total_points += 1
        if self.voxel_size <= 0:
            self.voxels[self.total_points] = (x, y, z, intensity)
            return

        key = (
            math.floor(x / self.voxel_size),
            math.floor(y / self.voxel_size),
            math.floor(z / self.voxel_size),
        )
        old = self.voxels.get(key)
        if old is None or intensity > old[3]:
            self.voxels[key] = (x, y, z, intensity)

    def save(self, output_path):
        save_pcd_with_intensity(self.voxels.values(), output_path)
        print(
            f"[PCD] 原始有效点: {self.total_points}, "
            f"选点显示点: {len(self.voxels)}, 体素: {self.voxel_size:.3f} m"
        )


class OptionalFullPcdWriter:
    def __init__(self, output_path, enabled):
        self.output_path = output_path
        self.enabled = enabled
        self.body_path = f"{output_path}.tmp_points"
        self.count = 0
        self.file_obj = None

    def __enter__(self):
        if self.enabled:
            self.file_obj = open(self.body_path, "w")
        return self

    def write(self, x, y, z, intensity):
        if not self.enabled:
            return
        self.file_obj.write(f"{x} {y} {z} {intensity}\n")
        self.count += 1

    def __exit__(self, exc_type, exc, tb):
        if self.file_obj is not None:
            self.file_obj.close()
        if not self.enabled:
            return
        if exc_type is not None:
            try:
                os.remove(self.body_path)
            except OSError:
                pass
            return

        with open(self.output_path, "w") as out, open(self.body_path, "r") as body:
            write_pcd_header(out, self.count)
            for line in body:
                out.write(line)
        os.remove(self.body_path)
        print(f"[PCD] 保存完整点云到: {self.output_path}")


# ===================== ROS2 bag 读取工具 =====================

def open_ros2_bag_reader(bag_file):
    """打开 ROS2 rosbag2，bag_file 可以是 .db3 文件或包含 metadata.yaml 的目录。"""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    return reader


def get_ros2_topic_types(bag_file):
    reader = open_ros2_bag_reader(bag_file)
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def ros2_type_to_class(type_name):
    if type_name == "sensor_msgs/msg/PointCloud2":
        return PointCloud2
    if get_message is None:
        raise RuntimeError(f"无法解析消息类型 {type_name}: rosidl_runtime_py 不可用")
    return get_message(type_name)


def find_first_topic_by_type(topic_types, type_names, preferred_topics=()):
    for topic in preferred_topics:
        if topic_types.get(topic) in type_names:
            return topic

    for topic, type_name in topic_types.items():
        if type_name in type_names:
            return topic

    return None

# ===================== 情况 1：PointCloud2 =====================

def find_intensity_field(msg):
    """在 PointCloud2 的 fields 中自动检测强度字段名称"""
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/livox/lidar",                        # 如有不同，可改成 topic 名称
    pcd_name="sensor_PointCloud2_inten_ascii.pcd"
):
    """
    将 rosbag 中 PointCloud2 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    print(f"[Bag] 打开 rosbag: {bag_file}")
    reader = open_ros2_bag_reader(bag_file)
    topic_types = get_ros2_topic_types(bag_file)

    if topic_name not in topic_types:
        print(f"[ERROR] bag 中不存在 topic: {topic_name}", file=sys.stderr)
        print(f"[ERROR] 可用 topics: {', '.join(topic_types.keys())}", file=sys.stderr)
        return None

    if topic_types[topic_name] != "sensor_msgs/msg/PointCloud2":
        print(
            f"[ERROR] topic {topic_name} 类型不是 PointCloud2，而是 {topic_types[topic_name]}",
            file=sys.stderr
        )
        return None

    # 1) 先检测强度字段
    intensity_field = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, PointCloud2)
            intensity_field = find_intensity_field(msg)
            if intensity_field:
                print(f"[Bag] 检测到 intensity 字段: {intensity_field}")
            break

    if not intensity_field:
        print("[ERROR] 未找到强度字段! 退出 PointCloud2 转换。", file=sys.stderr)
        return None

    # 2) 读取指定 topic 的所有点云，默认只保留体素降采样后的选点显示点云。
    preview_path = os.path.join(output_dir, pcd_name)
    full_path = os.path.join(output_dir, f"full_{pcd_name}")
    preview = VoxelPreviewBuffer(PREVIEW_VOXEL_SIZE)

    print(f"[Bag] 开始从 topic '{topic_name}' 读取 PointCloud2 点云...")
    if SAVE_FULL_PCD:
        print(f"[Bag] FAST_CALIB_SAVE_FULL_PCD=1，将额外写出完整 PCD: {full_path}")

    reader = open_ros2_bag_reader(bag_file)
    with OptionalFullPcdWriter(full_path, SAVE_FULL_PCD) as full_writer:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == topic_name:
                try:
                    msg = deserialize_message(data, PointCloud2)
                    field_names = ["x", "y", "z", intensity_field]
                    for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                        x, y, z, intensity = point[0], point[1], point[2], point[3]
                        preview.add(float(x), float(y), float(z), float(intensity))
                        full_writer.write(x, y, z, intensity)
                except Exception as e:
                    print(f"[ERROR] 读取错误: {str(e)}", file=sys.stderr)
                    continue

    if not preview.voxels:
        print("[ERROR] 未找到 PointCloud2 点云数据！", file=sys.stderr)
        return None

    preview.save(preview_path)
    return preview_path

# ===================== 情况 2：Livox CustomMsg =====================

def parse_livox_custom_msg(msg):
    """
    从 livox_ros_driver/CustomMsg 中解析 x, y, z, reflectivity
    假设 msg.points 是 CustomPoint 对象列表，字段为 x, y, z, reflectivity
    """
    points = []
    intensities = []

    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)

    return points, intensities

def convert_livox_custom_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/livox/lidar",                     # 如有不同，可改成 topic 名称
    pcd_name="livox_CustomMsg_inten_ascii.pcd"
):
    """
    将 rosbag 中 livox_ros_driver/CustomMsg 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    print(f"[Bag] 打开 rosbag: {bag_file}")
    reader = open_ros2_bag_reader(bag_file)
    topic_types = get_ros2_topic_types(bag_file)

    if topic_name not in topic_types:
        print(f"[ERROR] bag 中不存在 topic: {topic_name}", file=sys.stderr)
        print(f"[ERROR] 可用 topics: {', '.join(topic_types.keys())}", file=sys.stderr)
        return None

    msg_cls = ros2_type_to_class(topic_types[topic_name])

    preview_path = os.path.join(output_dir, pcd_name)
    full_path = os.path.join(output_dir, f"full_{pcd_name}")
    preview = VoxelPreviewBuffer(PREVIEW_VOXEL_SIZE)

    print(f"[Bag] 开始从 topic '{topic_name}' 读取 CustomMsg 点云...")
    if SAVE_FULL_PCD:
        print(f"[Bag] FAST_CALIB_SAVE_FULL_PCD=1，将额外写出完整 PCD: {full_path}")

    with OptionalFullPcdWriter(full_path, SAVE_FULL_PCD) as full_writer:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == topic_name:
                msg = deserialize_message(data, msg_cls)
                for pt in msg.points:
                    x = float(pt.x)
                    y = float(pt.y)
                    z = float(pt.z)
                    intensity = float(pt.reflectivity)
                    preview.add(x, y, z, intensity)
                    full_writer.write(x, y, z, intensity)

    if not preview.voxels:
        print("[ERROR] 未找到 Livox CustomMsg 点云数据!", file=sys.stderr)
        return None

    preview.save(preview_path)
    return preview_path

# ===================== 自动检测：这个 bag 用哪种方式 =====================

def detect_lidar_msg_type(bag_file):
    """
    在 bag 里扫一圈，检测是否有 PointCloud2 或 Livox CustomMsg。
    返回：
        "PointCloud2", "CustomMsg", 或 None
    如果两种都有，默认优先 PointCloud2,并打印提示。
    """
    has_pc2 = False
    has_livox = False

    print(f"[Detect] 扫描 bag: {bag_file}")
    topic_types = get_ros2_topic_types(bag_file)

    for topic, type_name in topic_types.items():
        if type_name == "sensor_msgs/msg/PointCloud2":
            has_pc2 = True
        elif type_name in (
            "livox_ros_driver/msg/CustomMsg",
            "livox_ros_driver2/msg/CustomMsg",
            "livox_ros_driver/CustomMsg",
        ):
            has_livox = True

        if has_pc2 and has_livox:
            break

    if has_pc2 and has_livox:
        print("[Detect] 同时检测到 PointCloud2 和 Livox CustomMsg, 默认使用 PointCloud2。")
        return "PointCloud2"
    elif has_pc2:
        print("[Detect] 检测到 PointCloud2 点云。")
        return "PointCloud2"
    elif has_livox:
        print("[Detect] 检测到 Livox CustomMsg 点云。")
        return "CustomMsg"
    else:
        print("[Detect] 未检测到 PointCloud2 或 Livox CustomMsg 点云。")
        return None

# ===================== Open3D 交互选点 & 保存范围 =====================

def select_and_save_points(pcd_folder, target_pcd_name):
    """
    在给定目录中读取指定 PCD 文件，用 Open3D 交互式选点并保存范围。
    """
    if o3d is None:
        print("[ERROR] 未安装 open3d，无法打开交互式选点窗口。", file=sys.stderr)
        print("[ERROR] 请在当前 Python 环境安装: python -m pip install open3d", file=sys.stderr)
        return

    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] 指定的 PCD 文件不存在: {pcd_path}", file=sys.stderr)
        return

    # 读取点云
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} 中没有点云数据，已跳过", file=sys.stderr)
        return

    print(f"\n正在处理: {target_pcd_name}")
    print("请在可视化窗口中按住 Shift 用鼠标左键选择点(至少4个)，然后按 Q 键关闭窗口")

    # 创建可视化窗口并添加点云
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"选择点 - {target_pcd_name}")
    vis.add_geometry(pcd)

    # 等待用户交互（Shift+左键选点, Q 退出）
    vis.run()
    vis.destroy_window()

    # 获取用户选择的点的索引
    selected_indices = vis.get_picked_points()

    if not selected_indices:
        print(f"[ERROR] 未选择任何点，{target_pcd_name} 没有保存文件", file=sys.stderr)
        return

    if len(selected_indices) < 4:
        print(f"[ERROR] 只选中了 {len(selected_indices)} 个点，少于 4 个，跳过该文件", file=sys.stderr)
        return

    # 只取前 4 个点
    selected_indices = selected_indices[:4]

    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]  # 形状 (4, 3)

    # 计算四个点在各轴上的最小值和最大值
    mins = selected_points.min(axis=0)  # [x_min_raw, y_min_raw, z_min_raw]
    maxs = selected_points.max(axis=0)  # [x_max_raw, y_max_raw, z_max_raw]

    # 按你的定义扩展 0.2m
    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    # 生成保存文件名 (与 PCD 文件同名，改为 txt)
    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] 已保存选点与范围到: {save_file}")
    print("点云处理完成。")

# ===================== main =====================

if __name__ == "__main__":
    # 1) 解析命令行参数：bag 路径 & 输出目录
    if len(sys.argv) > 1:
        bag_file = sys.argv[1]
    else:
        # 默认使用当前目录下的某个 bag，可以按需修改
        bag_file = os.path.join(os.getcwd(), "all_2025-11-17-18-22-27.bag")
        print(f"未指定 bag 文件，默认使用: {bag_file}")

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"未指定输出目录，使用当前目录: {output_dir}")

    if not os.path.isfile(bag_file):
        print(f"[ERROR] bag 文件 '{bag_file}' 不存在", file=sys.stderr)
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] 输出目录 '{output_dir}' 不存在", file=sys.stderr)
        sys.exit(1)

    # 不需要 rospy.init_node，完全离线工具

    topic_types = get_ros2_topic_types(bag_file)

    # 3) 自动检测 bag 中点云类型
    msg_type = detect_lidar_msg_type(bag_file)
    if msg_type is None:
        print("[ERROR] 未检测到支持的雷达消息类型，退出。", file=sys.stderr)
        sys.exit(1)

    # 4) 根据类型做对应的 PCD 转换
    if msg_type == "PointCloud2":
        lidar_topic = find_first_topic_by_type(
            topic_types,
            ["sensor_msgs/msg/PointCloud2"],
            preferred_topics=("/livox/lidar", "/hesai/pandar")
        )
        pcd_path = convert_pointcloud2_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name=lidar_topic,
            pcd_name="sensor_PointCloud2_inten_ascii.pcd"
        )
    else:  # "CustomMsg"
        lidar_topic = find_first_topic_by_type(
            topic_types,
            [
                "livox_ros_driver/msg/CustomMsg",
                "livox_ros_driver2/msg/CustomMsg",
                "livox_ros_driver/CustomMsg",
            ],
            preferred_topics=("/livox/lidar",)
        )
        pcd_path = convert_livox_custom_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name=lidar_topic,
            pcd_name="livox_CustomMsg_inten_ascii.pcd"
        )

    if pcd_path is None:
        print("[ERROR] PCD 生成失败，退出。", file=sys.stderr)
        sys.exit(1)

    # 5) 对刚生成的这个 PCD 做交互式选点 + 范围保存
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )
