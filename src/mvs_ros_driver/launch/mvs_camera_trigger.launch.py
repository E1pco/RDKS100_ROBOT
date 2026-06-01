"""Launch a single MVS camera node with trigger support."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mvs_ros_driver")
    default_config = os.path.join(pkg_share, "config", "left_camera_trigger.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to camera YAML config (OpenCV FileStorage format)",
    )

    camera_node = Node(
        package="mvs_ros_driver",
        executable="grabImgWithTrigger",
        name="mvs_camera_trigger",
        output="screen",
        arguments=[LaunchConfiguration("config")],
        respawn=True,
    )

    return LaunchDescription([config_arg, camera_node])
