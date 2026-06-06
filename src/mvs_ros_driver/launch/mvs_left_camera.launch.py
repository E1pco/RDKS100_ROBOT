"""Launch left MVS camera with runtime-adjustable parameters.

Exposure time, gain, and gamma are declared as ROS2 parameters
and can be changed at runtime without restarting the node:

    ros2 param set /left_camera exposure_time 8000
    ros2 param set /left_camera gain 10.0
    ros2 param set /left_camera gamma 1.2

Other parameters (SerialNumber, TriggerEnable, etc.) are read from
the OpenCV FileStorage config file at startup.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mvs_ros_driver")

    default_config = os.path.join(pkg_share, "config", "left_camera_trigger.yaml")
    default_params = os.path.join(pkg_share, "config", "left_camera_params.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to camera YAML config (OpenCV FileStorage format)",
    )
    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to ROS2 parameter YAML (exposure_time, gain, gamma)",
    )

    camera_node = Node(
        package="mvs_ros_driver",
        executable="grabImgWithTrigger",
        name="left_camera",
        output="screen",
        arguments=[LaunchConfiguration("config")],
        parameters=[LaunchConfiguration("params")],
        respawn=True,
    )

    return LaunchDescription([config_arg, params_arg, camera_node])
