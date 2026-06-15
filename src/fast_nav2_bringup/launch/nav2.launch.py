import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_nav2_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    ackermann_share = get_package_share_directory("ackermann_serial_bridge")

    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz_config = os.path.join(
        nav2_share,
        "rviz",
        "nav2_default_view.rviz",
    )
    default_bridge_config = os.path.join(
        ackermann_share, "config", "ackermann_serial_bridge.yaml"
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to a Nav2 occupancy map yaml file.",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the Nav2 parameters file.",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")
    launch_base_arg = DeclareLaunchArgument(
        "launch_base",
        default_value="true",
        description="Start the Ackermann serial bridge.",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz with the Nav2 view.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Full path to the RViz config file.",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": "False",
        }.items(),
    )

    serial_bridge = Node(
        condition=IfCondition(LaunchConfiguration("launch_base")),
        package="ackermann_serial_bridge",
        executable="serial_bridge_node",
        name="ackermann_serial_bridge",
        output="screen",
        parameters=[default_bridge_config, {"publish_tf": True}],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
        ],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(
        [
            map_arg,
            params_arg,
            use_sim_time_arg,
            autostart_arg,
            launch_base_arg,
            use_rviz_arg,
            rviz_config_arg,
            nav2,
            serial_bridge,
            rviz,
        ]
    )
