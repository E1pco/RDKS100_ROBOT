"""Launch file for the keyboard teleop node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ackermann_serial_bridge")
    default_config = os.path.join(pkg_share, "config", "keyboard_teleop.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to the keyboard teleop parameter YAML file",
    )

    teleop_node = Node(
        package="ackermann_serial_bridge",
        executable="keyboard_teleop",
        name="keyboard_teleop",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
        ],
    )

    return LaunchDescription([
        config_arg,
        teleop_node,
    ])
