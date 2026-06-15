import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_nav2_bringup")
    default_params = os.path.join(pkg_share, "config", "pointcloud_to_scan.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the pointcloud_to_laserscan parameters file.",
    )
    custom_topic_arg = DeclareLaunchArgument(
        "custom_topic",
        default_value="/livox/lidar",
        description="Input Livox CustomMsg topic.",
    )
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/livox/points",
        description="Intermediate or input PointCloud2 topic.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="Output LaserScan topic.",
    )
    lidar_frame_arg = DeclareLaunchArgument(
        "lidar_frame",
        default_value="livox_frame",
        description="Point cloud frame. Use livox if your driver publishes that frame.",
    )
    publish_base_tf_arg = DeclareLaunchArgument(
        "publish_base_tf",
        default_value="true",
        description="Publish identity TF from base_link to the LiDAR/IMU frame.",
    )
    convert_custom_arg = DeclareLaunchArgument(
        "convert_custom",
        default_value="true",
        description="Convert Livox CustomMsg to PointCloud2 before generating LaserScan.",
    )

    lidar_base_tf = Node(
        condition=IfCondition(LaunchConfiguration("publish_base_tf")),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_imu_tf",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "base_link",
            LaunchConfiguration("lidar_frame"),
        ],
    )

    custom_to_pointcloud = Node(
        condition=IfCondition(LaunchConfiguration("convert_custom")),
        package="fast_nav2_bringup",
        executable="livox_custom_to_pointcloud2.py",
        name="livox_custom_to_pointcloud2",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("custom_topic"),
                "output_topic": LaunchConfiguration("cloud_topic"),
                "frame_id": LaunchConfiguration("lidar_frame"),
            }
        ],
    )

    scan_converter = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("cloud_in", LaunchConfiguration("cloud_topic")),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
    )

    return LaunchDescription(
        [
            params_arg,
            custom_topic_arg,
            cloud_topic_arg,
            scan_topic_arg,
            lidar_frame_arg,
            publish_base_tf_arg,
            convert_custom_arg,
            lidar_base_tf,
            custom_to_pointcloud,
            scan_converter,
        ]
    )
