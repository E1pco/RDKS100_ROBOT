"""
ROS2 node: CmdVelRepublisher

Republishes the last received Twist command at a fixed high rate.
Bridges low-frequency command sources (teleop, nav2, topic pub)
to the high-frequency serial bridge.
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist


class CmdVelRepublisher(Node):
    """Republishes /cmd_vel at a configurable fixed rate."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_republisher")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("input_topic", "/cmd_vel_in")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("timeout", 2.0)

        self._rate: float = self.get_parameter("rate").value
        self._input_topic: str = self.get_parameter("input_topic").value
        self._output_topic: str = self.get_parameter("output_topic").value
        self._timeout: float = self.get_parameter("timeout").value

        # ------------------------------------------------------------------
        # Pub / Sub
        # ------------------------------------------------------------------
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(Twist, self._output_topic, qos)
        self._sub = self.create_subscription(Twist, self._input_topic, self._cb, qos)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._last_cmd: Optional[Twist] = None
        self._last_time: float = 0.0

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        period = 1.0 / max(self._rate, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"CmdVelRepublisher started: "
            f"{self._input_topic} -> {self._output_topic} @ {self._rate} Hz, "
            f"timeout={self._timeout}s"
        )

    # ==================================================================
    # Callbacks
    # ==================================================================

    def _cb(self, msg: Twist) -> None:
        """Store the latest incoming command."""
        self._last_cmd = msg
        self._last_time = time.monotonic()

    def _tick(self) -> None:
        """Timer callback: publish the last command at the fixed rate."""
        now = time.monotonic()

        if self._last_cmd is not None and (now - self._last_time) < self._timeout:
            cmd = self._last_cmd
        else:
            # No command or timed out — send zero velocity
            cmd = Twist()

        self._pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
