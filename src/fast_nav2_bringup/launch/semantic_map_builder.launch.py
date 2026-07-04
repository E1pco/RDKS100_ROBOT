import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_nav2_bringup")
    default_params = os.path.join(pkg_share, "config", "semantic_map_builder.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to semantic map builder parameters.",
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/cloud_registered",
        description="FAST-LIVO point cloud topic in the global frame.",
    )
    semantic_topic_arg = DeclareLaunchArgument(
        "semantic_topic",
        default_value="/perception/segmentation/edgesam",
        description="ai_msgs/PerceptionTargets topic from EdgeSAM or DOSOD.",
    )
    save_path_arg = DeclareLaunchArgument(
        "save_path",
        default_value="/home/sunrise/fast_ws/src/fast_nav2_bringup/maps/semantic_map.json",
        description="Output JSON file for the semantic voxel map.",
    )

    builder = Node(
        package="fast_nav2_bringup",
        executable="semantic_map_builder.py",
        name="semantic_map_builder",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "semantic_topic": LaunchConfiguration("semantic_topic"),
                "save_path": LaunchConfiguration("save_path"),
            },
        ],
    )

    return LaunchDescription(
        [
            params_arg,
            pointcloud_topic_arg,
            semantic_topic_arg,
            save_path_arg,
            builder,
        ]
    )
