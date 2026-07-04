"""
Camera node: captures frames from a USB camera and publishes them
as sensor_msgs/Image on a ROS2 topic.

Run:
  ros2 run arm camera_node --ros-args -p device:=/dev/video0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        # Open camera
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera: {device}')
            raise RuntimeError(f'Cannot open camera: {device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.get_logger().info(
            f'Camera opened: {device} {width}x{height}@{self.fps}fps'
        )

        # Publisher
        self.br = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._capture)
        self.get_logger().info(f'Publishing /camera/image_raw at {self.fps} Hz')

    def _capture(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down camera...')
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
