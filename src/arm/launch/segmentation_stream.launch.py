"""Launch DOSOD + EdgeSAM segmentation. Camera stream is expected on /camera/image_raw."""

import os
import signal
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


CLEANUP_TOKENS = (
    'ros2 launch arm segmentation_stream.launch.py',
    '/install/mono_edgesam/lib/mono_edgesam/mono_edgesam',
    '/install/hobot_dosod/lib/hobot_dosod/hobot_dosod',
    '/install/arm/lib/arm/dosod_filter_node',
    '/opt/tros/humble/lib/websocket/websocket',
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
        return [LogInfo(msg='Cleaned previous segmentation processes: ' + ', '.join(map(str, pids)))]
    return [LogInfo(msg='No previous segmentation processes found')]


def generate_launch_description():
    mono_edgesam_share = get_package_share_directory('mono_edgesam')
    segmentation_launch_file = os.path.join(
        mono_edgesam_share, 'launch', 's100', 'sam_with_dosod.launch.py')
    if not os.path.exists(segmentation_launch_file):
        segmentation_launch_file = os.path.join(
            mono_edgesam_share, 'launch', 'sam_with_dosod.launch.py')
    if not os.path.exists(segmentation_launch_file):
        segmentation_launch_file = '/home/sunrise/fast_ws/src/mono_edgesam/launch/s100/sam_with_dosod.launch.py'

    segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(segmentation_launch_file),
        launch_arguments={
            'sam_ros_img_sub_topic_name': '/camera/image_raw',
            'sam_image_width': '640',
            'sam_image_height': '480',
            'sam_codec_in_format': 'bgr8',
            'sam_max_rois': '0',
            'sam_cache_len_limit': '1',
            'sam_websocket_output_fps': '10',
            'sam_websocket_image_topic': '/segmentation/image_jpeg',
            'sam_enable_web_codec': 'false',
        }.items(),
    )

    return LaunchDescription([
        OpaqueFunction(function=_cleanup_previous),
        SetEnvironmentVariable('CAM_TYPE', 'ros'),
        segmentation_launch,
    ])
