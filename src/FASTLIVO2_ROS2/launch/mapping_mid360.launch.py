#!/usr/bin/python3
# -- coding: utf-8 --

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fast_livo")
    config_dir = os.path.join(package_share, "config")
    rviz_config_file = os.path.join(package_share, "rviz_cfg", "fast_livo2.rviz")

    default_avia_config = os.path.join(config_dir, "mid360_mvs.yaml")
    default_camera_config = os.path.join(config_dir, "camera_mvs.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Whether to launch RViz2.",
    )
    avia_config_arg = DeclareLaunchArgument(
        "avia_params_file",
        default_value=default_avia_config,
        description="FAST-LIVO MID360 + MVS parameter file.",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=default_camera_config,
        description="MVS camera calibration parameter file.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=rviz_config_file,
        description="RViz config file.",
    )
    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn fastlivo_mapping if it exits.",
    )

    avia_params_file = LaunchConfiguration("avia_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    rviz_config = LaunchConfiguration("rviz_config_file")
    use_respawn = LaunchConfiguration("use_respawn")

    shm_env_actions = [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            "/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml",
        ),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "1"),
        SetEnvironmentVariable("ROS_DISABLE_LOANED_MESSAGES", "0"),
    ]

    return LaunchDescription(
        shm_env_actions
        + [
            use_rviz_arg,
            avia_config_arg,
            camera_config_arg,
            rviz_config_arg,
            use_respawn_arg,
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="laserMapping",
                parameters=[avia_params_file, camera_params_file],
                output="screen",
                respawn=use_respawn,
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
