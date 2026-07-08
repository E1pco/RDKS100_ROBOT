import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_nav2_bringup")
    default_params = os.path.join(pkg_share, "config", "pcd_relocalization.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to PCD relocalization parameters.",
    )
    map_pcd_arg = DeclareLaunchArgument(
        "map_pcd",
        default_value="/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/Log/PCD/all_downsampled_points.pcd",
        description="Saved FAST-LIVO PCD map used as the relocalization target.",
    )
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/cloud_registered",
        description="Live FAST-LIVO PointCloud2 topic.",
    )

    relocalization = Node(
        package="fast_nav2_bringup",
        executable="pcd_relocalization_node.py",
        name="pcd_relocalization",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "map_pcd": LaunchConfiguration("map_pcd"),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
            },
        ],
    )

    return LaunchDescription(
        [
            params_arg,
            map_pcd_arg,
            cloud_topic_arg,
            relocalization,
        ]
    )
