#!/usr/bin/python3
# Lightweight FAST-LIVO2 launch for MID360 + MVS on RDK S100.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_livo")
    config_dir = os.path.join(pkg_share, "config")
    rviz_config_file = os.path.join(pkg_share, "rviz_cfg", "fast_livo2.rviz")

    avia_config_arg = DeclareLaunchArgument(
        "avia_params_file",
        default_value=os.path.join(config_dir, "mid360_mvs_light.yaml"),
        description="Lightweight FAST-LIVO2 parameter file",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=os.path.join(config_dir, "camera_mvs.yaml"),
        description="MVS camera intrinsic parameter file",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=rviz_config_file,
        description="RViz2 config file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch RViz2. Default false to reduce memory pressure.",
    )

    shm_env_actions = [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", "/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml"),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "1"),
        SetEnvironmentVariable("ROS_DISABLE_LOANED_MESSAGES", "0"),
    ]

    fastlivo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            LaunchConfiguration("avia_params_file"),
            LaunchConfiguration("camera_params_file"),
        ],
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        output="screen",
    )

    return LaunchDescription(shm_env_actions + [
        avia_config_arg,
        camera_config_arg,
        rviz_config_arg,
        use_rviz_arg,
        fastlivo_node,
        rviz_node,
    ])
