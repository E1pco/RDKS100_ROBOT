"""S100 robot core launch.

Starts always-on robot services only: chassis serial bridge and the feature manager.
Optional video, segmentation and arm follower processes are managed by
s100_feature_manager after receiving /control_console/feature_command messages.
"""

import os
import signal
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CORE_CLEANUP_TOKENS = (
    'ros2 launch arm arm_bringup.launch.py',
    'ros2 launch arm s100_robot_core.launch.py',
    '/install/ackermann_serial_bridge/lib/ackermann_serial_bridge/serial_bridge_node',
    '/install/arm/lib/arm/feature_manager_node',
    '/install/arm/lib/arm/rgb_img_republisher_node',
    'ros2 launch livox_ros_driver2 msg_MID360_launch.py',
    'ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py',
    'ros2 launch fast_livo mapping_avia.launch.py',
)


def _read_cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\x00', b' ').decode('utf-8', 'ignore').strip()
    except OSError:
        return ''


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _cleanup_previous_core(context, *args, **kwargs):
    current = os.getpid()
    pids = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == current:
            continue
        cmd = _read_cmdline(pid)
        if cmd and any(token in cmd for token in CORE_CLEANUP_TOKENS):
            pids.append(pid)
    pids = sorted(set(pids))
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.time() + 2.0
    while time.time() < deadline and any(_alive(pid) for pid in pids):
        time.sleep(0.1)
    for pid in pids:
        if _alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    if pids:
        return [LogInfo(msg='Cleaned previous S100 core processes: ' + ', '.join(map(str, pids)))]
    return [LogInfo(msg='No previous S100 core processes found')]


def generate_launch_description() -> LaunchDescription:
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    chassis_port_arg = DeclareLaunchArgument('chassis_port', default_value='/dev/chassis')
    arm_port_arg = DeclareLaunchArgument('arm_port', default_value='/dev/arm')
    arm_speed_arg = DeclareLaunchArgument('arm_speed', default_value='1200')

    ackermann_share = get_package_share_directory('ackermann_serial_bridge')
    chassis_config = os.path.join(ackermann_share, 'config', 'ackermann_serial_bridge.yaml')

    chassis_node = Node(
        package='ackermann_serial_bridge',
        executable='serial_bridge_node',
        name='ackermann_serial_bridge',
        output='screen',
        parameters=[chassis_config, {'port': LaunchConfiguration('chassis_port')}],
    )


    rgb_republisher = Node(
        package='arm',
        executable='rgb_img_republisher_node',
        name='rgb_img_republisher',
        output='screen',
        parameters=[{
            'input_topic': '/left_camera/image',
            'output_topic': '/rgb_img',
        }],
    )

    feature_manager = Node(
        package='arm',
        executable='feature_manager_node',
        name='s100_feature_manager',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('device'),
            'arm_port': LaunchConfiguration('arm_port'),
            'arm_speed': LaunchConfiguration('arm_speed'),
        }],
    )

    return LaunchDescription([
        OpaqueFunction(function=_cleanup_previous_core),
        device_arg,
        chassis_port_arg,
        arm_port_arg,
        arm_speed_arg,
        SetEnvironmentVariable('CAM_TYPE', 'ros'),
        chassis_node,
        feature_manager,
        rgb_republisher,
    ])
