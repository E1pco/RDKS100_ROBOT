from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vid", default_value="10374"),
            DeclareLaunchArgument("pid", default_value="26"),
            DeclareLaunchArgument("poll_rate_hz", default_value="10.0"),
            Node(
                package="respeaker_xvf3800",
                executable="doa_node",
                name="respeaker_xvf3800_doa",
                output="screen",
                parameters=[
                    {
                        "vid": ParameterValue(LaunchConfiguration("vid"), value_type=int),
                        "pid": ParameterValue(LaunchConfiguration("pid"), value_type=int),
                        "poll_rate_hz": ParameterValue(LaunchConfiguration("poll_rate_hz"), value_type=float),
                    }
                ],
            ),
        ]
    )
