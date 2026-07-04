"""
Follower node: subscribes to /arm/joints (JointState) and mirrors
the leader's servo positions to the follower arm with low-pass filtering.

Run:
  ros2 run arm follower_node --ros-args -p port:=/dev/ttyACM1
"""

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .driver.ftservo_controller import ServoController

DEFAULT_JOINTS = (
    'shoulder_pan,shoulder_lift,elbow_flex,'
    'wrist_flex,wrist_yaw,wrist_roll,gripper'
)


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/arm')
        self.declare_parameter('baudrate', 1_000_000)
        self.declare_parameter('speed', 1200)
        self.declare_parameter('alpha', 0.35)
        self.declare_parameter('joints', DEFAULT_JOINTS)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.speed = self.get_parameter('speed').value
        self.alpha = min(max(self.get_parameter('alpha').value, 0.0), 1.0)
        joints_str = self.get_parameter('joints').value

        self.joint_names = [n.strip() for n in joints_str.split(',') if n.strip()]
        if not self.joint_names:
            self.get_logger().error('No joint names specified!')
            raise ValueError('joint names cannot be empty')

        # Low-pass filter state
        self.filtered = {}

        # Config file path (relative to this package)
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(pkg_dir, 'driver', 'right_arm.json')

        # Connect to follower arm
        self.get_logger().info(f'Connecting to follower arm on {port} @ {baudrate}...')
        self.controller = ServoController(port, baudrate, config_path)
        self.get_logger().info(f'Follower arm connected. Joints: {self.joint_names}')

        # Subscriber
        self.sub = self.create_subscription(
            JointState, '/arm/joints', self._on_joint_state, 10
        )
        self.get_logger().info(
            f'Subscribed to /arm/joints. alpha={self.alpha}, speed={self.speed}'
        )

    def _on_joint_state(self, msg: JointState):
        # Build position dict from the message
        name_to_pos = dict(zip(msg.name, msg.position))

        targets = {}
        for name in self.joint_names:
            if name not in name_to_pos:
                continue

            raw = int(round(name_to_pos[name]))

            # Low-pass filter (same logic as teleop_leader_follower.py)
            if name in self.filtered:
                smoothed = self.filtered[name] + self.alpha * (raw - self.filtered[name])
            else:
                smoothed = float(raw)

            self.filtered[name] = smoothed
            targets[name] = int(round(smoothed))

        if targets:
            self.controller.fast_move_to_pose(targets, speed=self.speed)

    def destroy_node(self):
        self.get_logger().info('Shutting down, closing serial port...')
        self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
