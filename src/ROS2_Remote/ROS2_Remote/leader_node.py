"""
Leader node: reads servo positions from the leader arm and publishes them
as a sensor_msgs/JointState topic at a fixed rate.

Run:
  ros2 run ROS2_Remote leader_node --ros-args -p port:=/dev/ttyACM0
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


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1_000_000)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('joints', DEFAULT_JOINTS)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        rate = self.get_parameter('rate').value
        joints_str = self.get_parameter('joints').value

        self.joint_names = [n.strip() for n in joints_str.split(',') if n.strip()]
        if not self.joint_names:
            self.get_logger().error('No joint names specified!')
            raise ValueError('joint names cannot be empty')

        # Config file path (relative to this package)
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(pkg_dir, 'driver', 'right_arm_leader.json')

        # Connect to leader arm
        self.get_logger().info(f'Connecting to leader arm on {port} @ {baudrate}...')
        self.controller = ServoController(port, baudrate, config_path)
        self.get_logger().info(f'Leader arm connected. Joints: {self.joint_names}')

        # Publisher
        self.pub = self.create_publisher(JointState, '/arm/joints', 10)

        # Timer for periodic publishing
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._publish_positions)
        self.get_logger().info(f'Publishing at {rate} Hz (period={period:.4f}s)')

    def _publish_positions(self):
        try:
            positions = self.controller.read_servo_positions(self.joint_names)
        except Exception as e:
            self.get_logger().warn(f'Read failed: {e}')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(positions[n]) for n in self.joint_names]
        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down, closing serial port...')
        self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
