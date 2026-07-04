"""Launch camera and JPEG stream for the web console."""

import os
import signal
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CLEANUP_TOKENS = (
    'ros2 launch arm video_stream.launch.py',
    '/install/arm/lib/arm/camera_node',
    '/opt/tros/humble/lib/hobot_codec',
    '/install/hobot_codec/lib/hobot_codec',
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


def _cleanup_previous(context, *args, **kwargs):
    current = os.getpid()
    pids = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == current:
            continue
        cmd = _read_cmdline(pid)
        if cmd and any(token in cmd for token in CLEANUP_TOKENS):
            pids.append(pid)
    pids = sorted(set(pids))
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.time() + 1.5
    while time.time() < deadline and any(_alive(pid) for pid in pids):
        time.sleep(0.1)
    for pid in pids:
        if _alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    if pids:
        return [LogInfo(msg='Cleaned previous video stream processes: ' + ', '.join(map(str, pids)))]
    return [LogInfo(msg='No previous video stream processes found')]


def generate_launch_description():
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')

    camera_node = Node(
        package='arm',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
        }],
    )

    codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_in_format': 'bgr8',
            'codec_out_mode': 'ros',
            'codec_out_format': 'jpeg',
            'codec_sub_topic': '/camera/image_raw',
            'codec_pub_topic': '/segmentation/image_jpeg',
        }.items(),
    )

    return LaunchDescription([OpaqueFunction(function=_cleanup_previous), device_arg, camera_node, codec_node])
