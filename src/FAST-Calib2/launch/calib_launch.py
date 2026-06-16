#!/usr/bin/env python3
"""
Launch file for FAST-Calib2 single-scene calibration (ROS2 version)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('fast_calib')

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2'
    )

    # Config file path
    config_file = PathJoinSubstitution([pkg_share, 'config', 'qr_params.yaml'])

    # Set LD_PRELOAD to use system libusb (fix PCL compatibility issue with MVS SDK)
    ld_preload = SetEnvironmentVariable(
        'LD_PRELOAD',
        '/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0'
    )

    # Main calibration node
    fast_calib_node = Node(
        package='fast_calib',
        executable='fast_calib',
        name='fast_calib',
        output='screen',
        parameters=[config_file],
    )

    # RViz2 node (conditional)
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'fast_livo2.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([
        ld_preload,
        rviz_arg,
        fast_calib_node,
        rviz_node,
    ])
