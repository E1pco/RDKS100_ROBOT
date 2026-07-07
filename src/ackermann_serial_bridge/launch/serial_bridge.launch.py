"""Launch file for the Ackermann serial bridge node.

Loads parameters from config/ackermann_serial_bridge.yaml and optionally
remaps topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ackermann_serial_bridge")
    source_config = os.path.expanduser("~/fast_ws/src/ackermann_serial_bridge/config/ackermann_serial_bridge.yaml")
    default_config = source_config if os.path.exists(source_config) else os.path.join(pkg_share, "config", "ackermann_serial_bridge.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to the bridge parameter YAML file",
    )

    bridge_node = Node(
        package="ackermann_serial_bridge",
        executable="serial_bridge_node",
        name="ackermann_serial_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
        ],
        remappings=[
            # Add topic remappings here if needed, e.g.:
            # ("/cmd_vel", "/ackermann_cmd"),
        ],
    )

    return LaunchDescription([
        config_arg,
        bridge_node,
    ])
