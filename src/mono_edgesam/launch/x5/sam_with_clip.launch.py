# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
CLIP检索 + DOSOD检测 + EdgeSAM分割 联合launch文件

Pipeline:
  image_folder_publisher (静态图片发布)
       ↓ /image_left_raw
  CLIP检索 (clip_manage mode=2, 文本→图片匹配)
       ↓ tros_queried_res (匹配结果)
  hobot_dosod (开放词汇检测)
       ↓ /hobot_dnn_detection (bounding boxes)
  EdgeSAM (实例分割)
       ↓ /perception/segmentation/edgesam (分割mask)

使用方法:
  # 分步运行 (推荐):
  # Step 1: 先索引图片到CLIP数据库
  ros2 launch mono_edgesam sam_with_clip.launch.py clip_mode:=0

  # Step 2: 启动完整管线 (检索+检测+分割)
  ros2 launch mono_edgesam sam_with_clip.launch.py clip_mode:=1 clip_text:="a cat"

  # 一步到位 (直接检测+分割，跳过CLIP索引):
  ros2 launch mono_edgesam sam_with_clip.launch.py clip_mode:=0 skip_clip:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    # ======================== Paths ========================
    sam_basic_path = os.path.join(
        get_package_prefix('mono_edgesam'),
        'lib/mono_edgesam/config')

    dosod_basic_path = os.path.join(
        get_package_prefix('hobot_dosod'),
        'lib/hobot_dosod/config')

    # CLIP web UI
    clip_index_path = os.path.join(
        get_package_prefix('clip_manage'),
        "lib/clip_manage/config/")
    cp_cmd = "cp -r " + clip_index_path + "index.html ."
    os.system(cp_cmd)

    scripts_path = os.path.join(
        get_package_prefix('mono_edgesam'),
        'lib/mono_edgesam/scripts')

    # ======================== Common Args ========================
    img_topic_launch_arg = DeclareLaunchArgument(
        "img_topic",
        default_value=TextSubstitution(text="/image_left_raw")
    )
    img_folder_launch_arg = DeclareLaunchArgument(
        "img_folder",
        default_value=TextSubstitution(
            text="/home/sunrise/fast_ws/test_images")
    )
    img_interval_ms_launch_arg = DeclareLaunchArgument(
        "img_interval_ms",
        default_value=TextSubstitution(text="500")
    )
    skip_clip_launch_arg = DeclareLaunchArgument(
        "skip_clip",
        default_value=TextSubstitution(text="false")
    )

    # ======================== CLIP Args ========================
    clip_mode_launch_arg = DeclareLaunchArgument(
        "clip_mode",
        default_value=TextSubstitution(text="0")  # 0=存储索引
    )
    clip_db_file_launch_arg = DeclareLaunchArgument(
        "clip_db_file",
        default_value=TextSubstitution(text="clip.db")
    )
    clip_text_launch_arg = DeclareLaunchArgument(
        "clip_text",
        default_value=TextSubstitution(text="a cat")
    )
    clip_storage_folder_launch_arg = DeclareLaunchArgument(
        "clip_storage_folder",
        default_value=TextSubstitution(text="/userdata/config")
    )
    clip_result_folder_launch_arg = DeclareLaunchArgument(
        "clip_result_folder",
        default_value=TextSubstitution(text="/userdata/result")
    )
    clip_image_model_file_name_launch_arg = DeclareLaunchArgument(
        "clip_image_model_file_name",
        default_value=TextSubstitution(text="config/full_model_11.bin")
    )
    clip_topk_launch_arg = DeclareLaunchArgument(
        "clip_topk",
        default_value=TextSubstitution(text="10")
    )

    # ======================== DOSOD Args ========================
    dosod_model_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_model_file_name",
        default_value=TextSubstitution(
            text="dosod_mlp3x_l_rep-int16.hbm")
    )
    dosod_vocabulary_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_vocabulary_file_name",
        default_value=TextSubstitution(text="offline_vocabulary.json")
    )
    dosod_score_threshold_launch_arg = DeclareLaunchArgument(
        "dosod_score_threshold",
        default_value=TextSubstitution(text="0.6")
    )

    # ======================== SAM Args ========================
    sam_msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "sam_msg_pub_topic_name",
        default_value=TextSubstitution(text="perception/segmentation/edgesam")
    )
    sam_encoder_model_file_name_launch_arg = DeclareLaunchArgument(
        "sam_encoder_model_file_name",
        default_value=TextSubstitution(text="edgesam_encoder_512.bin")
    )
    sam_decoder_model_file_name_launch_arg = DeclareLaunchArgument(
        "sam_decoder_model_file_name",
        default_value=TextSubstitution(text="edgesam_decoder_512.bin")
    )

    # ======================== Nodes ========================

    # --- 静态图片发布节点 ---
    # 从文件夹循环读取图片，发布到 /image_left_raw 话题
    img_publisher = Node(
        package='mono_edgesam',
        executable='image_folder_publisher.py',
        output='screen',
        parameters=[
            {"folder": LaunchConfiguration('img_folder')},
            {"topic": LaunchConfiguration('img_topic')},
            {"interval_ms": LaunchConfiguration('img_interval_ms')},
            {"loop": True},
            {"encoding": "rgb8"}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # --- CLIP Image Encoder (BPU推理) ---
    clip_encode_image = Node(
        package='clip_encode_image',
        executable='clip_encode_image',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_sync_mode": 1},
            {"model_file_name": LaunchConfiguration('clip_image_model_file_name')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # --- CLIP Text Encoder (ONNX CPU推理) ---
    clip_encode_text = Node(
        package='clip_encode_text',
        executable='clip_encode_text_node',
        output='screen',
        parameters=[
            {"feed_type": True},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # --- CLIP Manager (检索编排) ---
    clip_manage = Node(
        package='clip_manage',
        executable='clip_manage',
        output='screen',
        parameters=[
            {"mode": LaunchConfiguration('clip_mode')},
            {"db_file": LaunchConfiguration('clip_db_file')},
            {"text": LaunchConfiguration('clip_text')},
            {"storage_folder": LaunchConfiguration('clip_storage_folder')},
            {"result_folder": LaunchConfiguration('clip_result_folder')},
            {"topk": LaunchConfiguration('clip_topk')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # --- DOSOD 开放词汇检测 ---
    # 订阅图像话题，检测目标物体，输出bounding box到 /hobot_dnn_detection
    dosod_node = Node(
        package='hobot_dosod',
        executable='hobot_dosod',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_shared_mem_sub": 0},
            {"ros_img_sub_topic_name": LaunchConfiguration('img_topic')},
            {"roi": False},
            {"dump_raw_img": 0},
            {"dump_render_img": 0},
            {"ai_msg_pub_topic_name": "/hobot_dnn_detection"},
            {"model_file_name": [dosod_basic_path, "/", LaunchConfiguration(
                'dosod_model_file_name')]},
            {"vocabulary_file_name": [dosod_basic_path, "/", LaunchConfiguration(
                'dosod_vocabulary_file_name')]},
            {"trigger_mode": 0},
            {"class_mode": 0},
            {"score_threshold": LaunchConfiguration('dosod_score_threshold')}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # --- EdgeSAM 实例分割 ---
    # 订阅图像 + DOSOD的bounding box，进行逐box分割
    sam_node = Node(
        package='mono_edgesam',
        executable='mono_edgesam',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_regular_box": 0},       # 使用外部检测框
            {"is_padding_seg": 1},
            {"is_shared_mem_sub": 0},
            {"ros_img_sub_topic_name": LaunchConfiguration('img_topic')},
            {"ai_msg_sub_topic_name": "/hobot_dnn_detection"},
            {"ai_msg_pub_topic_name": LaunchConfiguration(
                'sam_msg_pub_topic_name')},
            {"encoder_model_file_name": [sam_basic_path, "/", LaunchConfiguration(
                "sam_encoder_model_file_name")]},
            {"decoder_model_file_name": [sam_basic_path, "/", LaunchConfiguration(
                "sam_decoder_model_file_name")]},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ======================== Launch ========================
    return LaunchDescription([
        # Common args
        img_topic_launch_arg,
        img_folder_launch_arg,
        img_interval_ms_launch_arg,
        skip_clip_launch_arg,
        # CLIP args
        clip_mode_launch_arg,
        clip_db_file_launch_arg,
        clip_text_launch_arg,
        clip_storage_folder_launch_arg,
        clip_result_folder_launch_arg,
        clip_image_model_file_name_launch_arg,
        clip_topk_launch_arg,
        # DOSOD args
        dosod_model_file_name_launch_arg,
        dosod_vocabulary_file_name_launch_arg,
        dosod_score_threshold_launch_arg,
        # SAM args
        sam_msg_pub_topic_name_launch_arg,
        sam_encoder_model_file_name_launch_arg,
        sam_decoder_model_file_name_launch_arg,
        # 图片发布节点
        img_publisher,
        # CLIP nodes
        clip_encode_image,
        clip_encode_text,
        clip_manage,
        # DOSOD node (检测)
        dosod_node,
        # EdgeSAM node (分割)
        sam_node,
    ])
