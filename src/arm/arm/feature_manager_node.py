#!/usr/bin/env python3
"""Runtime feature manager for optional robot functions on the S100 host."""

import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


WORKSPACE = '/home/sunrise/fast_ws'
ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = f'{WORKSPACE}/install/setup.bash'


def ros_command(command):
    return (
        'set +u; '
        f'source {ROS_SETUP}; '
        f'source {WS_SETUP}; '
        f'export CAM_TYPE=ros; '
        f'cd {WORKSPACE}; '
        f'exec {command}'
    )


def read_cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
    except OSError:
        return ''


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


class ManagedProcess:
    def __init__(self, name, command, tokens, log_path):
        self.name = name
        self.command = command
        self.tokens = tuple(tokens)
        self.log_path = log_path
        self.process = None

    def _matching_pids(self):
        current = os.getpid()
        parent = os.getppid()
        pids = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid in (current, parent):
                continue
            cmd = read_cmdline(pid)
            if cmd and any(token in cmd for token in self.tokens):
                pids.append(pid)
        return sorted(set(pids))

    def running(self):
        if self.process is not None and self.process.poll() is None:
            return True
        self.process = None
        return bool(self._matching_pids())

    def start(self, logger):
        self.stop(logger, quiet=True)
        log = open(self.log_path, 'ab', buffering=0)
        self.process = subprocess.Popen(
            ['bash', '-c', ros_command(self.command)],
            stdout=log,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            close_fds=True,
        )
        logger.info(f'started {self.name}: pid={self.process.pid}, log={self.log_path}')
        return True

    def stop(self, logger, quiet=False):
        pids = set(self._matching_pids())
        if self.process is not None and self.process.poll() is None:
            try:
                pids.add(os.getpgid(self.process.pid))
            except ProcessLookupError:
                pass
        if not pids:
            self.process = None
            return False
        for pid in sorted(pids):
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except OSError:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.time() + 2.0
        while time.time() < deadline and any(alive(pid) for pid in pids):
            time.sleep(0.1)
        for pid in sorted(pids):
            if alive(pid):
                try:
                    os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except OSError:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self.process = None
        if not quiet:
            logger.info(f'stopped {self.name}: pids={sorted(pids)}')
        return True


class FeatureManagerNode(Node):
    def __init__(self):
        super().__init__('s100_feature_manager')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('arm_port', '/dev/arm')
        self.declare_parameter('arm_speed', 1200)
        device = self.get_parameter('camera_device').value
        arm_port = self.get_parameter('arm_port').value
        arm_speed = int(self.get_parameter('arm_speed').value)

        self.processes = {
            'video': ManagedProcess(
                'video',
                f'ros2 launch arm video_stream.launch.py device:={device}',
                (
                    'ros2 launch arm video_stream.launch.py',
                    '/install/arm/lib/arm/camera_node',
                    '/opt/tros/humble/lib/hobot_codec',
                    '/install/hobot_codec/lib/hobot_codec',
                ),
                '/tmp/s100_video_stream.log',
            ),
            'segmentation': ManagedProcess(
                'segmentation',
                'ros2 launch arm segmentation_stream.launch.py',
                (
                    'ros2 launch arm segmentation_stream.launch.py',
                    '/install/mono_edgesam/lib/mono_edgesam/mono_edgesam',
                    '/install/hobot_dosod/lib/hobot_dosod/hobot_dosod',
                    '/install/arm/lib/arm/dosod_filter_node',
                    '/opt/tros/humble/lib/websocket/websocket',
                ),
                '/tmp/s100_segmentation_stream.log',
            ),
            'follower': ManagedProcess(
                'follower',
                f'ros2 run arm follower_node --ros-args -r __node:=follower_node -p port:={arm_port} -p speed:={arm_speed}',
                (
                    '/install/arm/lib/arm/follower_node',
                    'ros2 run arm follower_node',
                ),
                '/tmp/s100_follower.log',
            ),
            'lidar': ManagedProcess(
                'lidar',
                'ros2 launch livox_ros_driver2 msg_MID360_launch.py',
                (
                    'ros2 launch livox_ros_driver2 msg_MID360_launch.py',
                    '/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node',
                ),
                '/tmp/s100_lidar.log',
            ),
            'mvs_camera': ManagedProcess(
                'mvs_camera',
                'ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py',
                (
                    'ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py',
                    '/install/mvs_ros_driver/lib/mvs_ros_driver/grabImgWithTrigger',
                ),
                '/tmp/s100_mvs_camera.log',
            ),
            'fast_livo2': ManagedProcess(
                'fast_livo2',
                'ros2 launch fast_livo mapping_avia.launch.py '
                'avia_params_file:=/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/config/mid360_mvs.yaml '
                'camera_params_file:=/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/config/camera_mvs.yaml '
                'use_rviz:=0',
                (
                    'ros2 launch fast_livo mapping_avia.launch.py',
                    '/install/fast_livo/lib/fast_livo',
                    'laserMapping',
                ),
                '/tmp/s100_fast_livo2.log',
            ),
        }

        self.desired = {
            'video_stream': False,
            'segmentation': False,
            'arm_passthrough': False,
            'lidar': False,
            'fast_livo2': False,
        }
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/control_console/feature_status', status_qos)
        self.create_subscription(String, '/control_console/feature_command', self._on_control, 10)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('s100_feature_manager ready: listening on /control_console/feature_command')
        self.publish_status()

    def _sync_video_dependency(self):
        should_run = self.desired['video_stream'] or self.desired['segmentation']
        if should_run:
            if not self.processes['video'].running():
                self.processes['video'].start(self.get_logger())
        else:
            self.processes['video'].stop(self.get_logger())

    def _set_video(self, enable):
        self.desired['video_stream'] = enable
        if not enable and self.desired['segmentation']:
            self.desired['segmentation'] = False
            self.processes['segmentation'].stop(self.get_logger())
        self._sync_video_dependency()

    def _set_segmentation(self, enable):
        self.desired['segmentation'] = enable
        if enable:
            self._sync_video_dependency()
            self.processes['segmentation'].start(self.get_logger())
        else:
            self.processes['segmentation'].stop(self.get_logger())
            self._sync_video_dependency()

    def _set_arm(self, enable):
        self.desired['arm_passthrough'] = enable
        if enable:
            self.processes['follower'].start(self.get_logger())
        else:
            self.processes['follower'].stop(self.get_logger())

    def _set_lidar(self, enable):
        self.desired['lidar'] = enable
        if enable:
            self.processes['lidar'].start(self.get_logger())
        else:
            if self.desired.get('fast_livo2'):
                self.get_logger().info('lidar stop requested, keeping LiDAR alive because FAST-LIVO2 depends on it')
                return
            self.processes['lidar'].stop(self.get_logger())

    def _set_fast_livo2(self, enable):
        self.desired['fast_livo2'] = enable
        if enable:
            if not self.processes['lidar'].running():
                self.processes['lidar'].start(self.get_logger())
            if not self.processes['mvs_camera'].running():
                self.processes['mvs_camera'].start(self.get_logger())
            time.sleep(1.0)
            self.processes['fast_livo2'].start(self.get_logger())
        else:
            self.processes['fast_livo2'].stop(self.get_logger())
            self.processes['mvs_camera'].stop(self.get_logger())
            if not self.desired.get('lidar'):
                self.processes['lidar'].stop(self.get_logger())

    def _on_control(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'invalid feature command: {exc}')
            return
        feature = str(data.get('feature', ''))
        action = str(data.get('action', ''))
        enable = action == 'start'
        self.get_logger().info(f'feature command: {feature} {action}')
        try:
            if feature == 'video_stream':
                self._set_video(enable)
            elif feature == 'segmentation':
                self._set_segmentation(enable)
            elif feature == 'arm_passthrough':
                self._set_arm(enable)
            elif feature == 'lidar':
                self._set_lidar(enable)
            elif feature == 'fast_livo2':
                self._set_fast_livo2(enable)
            else:
                self.get_logger().warn(f'unknown feature: {feature}')
        finally:
            self.publish_status()

    def publish_status(self):
        video = self.processes['video'].running()
        segmentation = self.processes['segmentation'].running()
        arm = self.processes['follower'].running()
        lidar = self.processes['lidar'].running()
        fast_livo2 = self.processes['fast_livo2'].running()
        msg = String()
        msg.data = json.dumps({
            'host': '192.168.112.224',
            'features': {
                'video_stream': video,
                'segmentation': segmentation,
                'arm_passthrough': arm,
                'lidar': lidar,
                'fast_livo2': fast_livo2,
            },
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def destroy_node(self):
        for name in ('fast_livo2', 'mvs_camera', 'segmentation', 'video', 'follower', 'lidar'):
            self.processes[name].stop(self.get_logger(), quiet=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FeatureManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
