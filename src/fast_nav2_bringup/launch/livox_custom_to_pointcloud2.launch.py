from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/livox/lidar",
        description="Input Livox CustomMsg topic.",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/livox/points",
        description="Output PointCloud2 topic.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="",
        description="Override output frame. Empty keeps the input frame.",
    )

    converter = Node(
        package="fast_nav2_bringup",
        executable="livox_custom_to_pointcloud2.py",
        name="livox_custom_to_pointcloud2",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
            }
        ],
    )

    return LaunchDescription(
        [
            input_topic_arg,
            output_topic_arg,
            frame_id_arg,
            converter,
        ]
    )
