"""Launch two MVS camera nodes (left + right) with trigger support."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mvs_ros_driver")

    left_config = os.path.join(pkg_share, "config", "left_camera_trigger.yaml")
    right_config = os.path.join(pkg_share, "config", "right_camera_trigger.yaml")

    left_node = Node(
        package="mvs_ros_driver",
        executable="grabImgWithTrigger",
        name="left_camera",
        output="screen",
        arguments=[left_config],
        respawn=True,
    )

    right_node = Node(
        package="mvs_ros_driver",
        executable="grabImgWithTrigger",
        name="right_camera",
        output="screen",
        arguments=[right_config],
        respawn=True,
    )

    return LaunchDescription([left_node, right_node])
