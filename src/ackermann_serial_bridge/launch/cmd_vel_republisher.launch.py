"""Launch file for the cmd_vel republisher node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ackermann_serial_bridge")
    default_config = os.path.join(pkg_share, "config", "cmd_vel_republisher.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to the republisher parameter YAML file",
    )

    republisher_node = Node(
        package="ackermann_serial_bridge",
        executable="cmd_vel_republisher",
        name="cmd_vel_republisher",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
        ],
    )

    return LaunchDescription([
        config_arg,
        republisher_node,
    ])
