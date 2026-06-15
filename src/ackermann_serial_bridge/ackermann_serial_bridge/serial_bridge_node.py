"""
ROS2 node: AckermannSerialBridgeNode

Bridges a Ackermann chassis (STM32 serial protocol) to ROS2.

* Reads uplink status frames from the serial port and publishes:
    - /odom            (nav_msgs/Odometry)
    - /battery_state   (sensor_msgs/BatteryState)
    - /diagnostics     (diagnostic_msgs/DiagnosticArray)
    - TF  (optional)   odom → base_link
* Subscribes to /cmd_vel (geometry_msgs/Twist) and sends downlink frames.

IMU data in uplink frames is parsed but intentionally NOT published.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import serial  # pyserial

from .protocol import (
    HEADER,
    TAIL,
    UPLINK_FRAME_LEN,
    DOWNLINK_FRAME_LEN,
    build_downlink_frame,
    UplinkFrameParser,
)


class AckermannSerialBridgeNode(Node):
    """ROS2 ↔  Ackermann chassis serial bridge."""

    def __init__(self) -> None:
        super().__init__("ackermann_serial_bridge")
        # Allow parallel timer callbacks to avoid serial read/write starvation.
        self._callback_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # Declare & read parameters
        # ------------------------------------------------------------------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_send_rate", 20.0)
        self.declare_parameter("read_rate", 100.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("yaw_sign", -1.0)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        self._port: str = self.get_parameter("port").value
        self._baudrate: int = self.get_parameter("baudrate").value
        self._cmd_send_rate: float = self.get_parameter("cmd_send_rate").value
        self._read_rate: float = self.get_parameter("read_rate").value
        self._command_timeout: float = self.get_parameter("command_timeout").value
        self._max_linear_speed: float = self.get_parameter("max_linear_speed").value
        self._max_angular_speed: float = self.get_parameter("max_angular_speed").value
        self._yaw_sign: float = self.get_parameter("yaw_sign").value
        self._publish_odom: bool = self.get_parameter("publish_odom").value
        self._publish_tf: bool = self.get_parameter("publish_tf").value
        self._frame_id: str = self.get_parameter("frame_id").value
        self._base_frame_id: str = self.get_parameter("base_frame_id").value

        # ------------------------------------------------------------------
        # ROS  publishers / subscribers
        # ------------------------------------------------------------------
        # Best-effort QoS for sensor-like topics (matches common sensor drivers)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self._publish_odom:
            self._odom_pub = self.create_publisher(Odometry, "/odom", sensor_qos)

        self._battery_pub = self.create_publisher(
            BatteryState, "/battery_state", sensor_qos
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        self._cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._cmd_vel_cb,
            10,
            callback_group=self._callback_group,
        )

        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------------
        # Serial port
        # ------------------------------------------------------------------
        self._ser: Optional[serial.Serial] = None
        self._parser = UplinkFrameParser()
        self._open_serial()

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        # Odometry integration
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._prev_time: Optional[float] = None

        # Last received cmd_vel and timestamp
        self._last_cmd: Optional[Twist] = None
        self._last_cmd_time: float = 0.0

        # Diagnostics
        self._rx_count = 0
        self._tx_count = 0
        self._checksum_errors = 0
        self._parse_errors = 0
        self._last_frame_time: float = 0.0
        self._last_voltage: float = 0.0
        self._motor_enabled: bool = True

        # Throttle log counter
        self._tick = 0

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        read_period = 1.0 / max(self._read_rate, 1.0)
        self._read_timer = self.create_timer(
            read_period,
            self._read_serial_cb,
            callback_group=self._callback_group,
        )

        cmd_period = 1.0 / max(self._cmd_send_rate, 1.0)
        self._cmd_timer = self.create_timer(
            cmd_period,
            self._send_cmd_cb,
            callback_group=self._callback_group,
        )

        self._diag_timer = self.create_timer(
            1.0,
            self._publish_diagnostics_cb,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"AckermannSerialBridge started: port={self._port} "
            f"baud={self._baudrate} yaw_sign={self._yaw_sign}"
        )

    # ==================================================================
    # Serial port management
    # ==================================================================

    def _open_serial(self) -> None:
        """Open (or re-open) the serial port.  Never raises to caller."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,          # non-blocking read
                write_timeout=0.5,  # short write timeout
            )
            self._parser.reset()
            self.get_logger().info(f"Serial port {self._port} opened.")
        except Exception as exc:
            self._ser = None
            self.get_logger().warn(f"Cannot open serial port {self._port}: {exc}")

    # ==================================================================
    # Uplink — read & publish
    # ==================================================================

    def _read_serial_cb(self) -> None:
        """Timer callback: read bytes from serial, parse uplink frames."""
        if self._ser is None or not self._ser.is_open:
            # Try to reconnect periodically
            self._open_serial()
            return

        try:
            data = self._ser.read(512)
        except Exception as exc:
            self.get_logger().warn(f"Serial read error: {exc}")
            self._parse_errors += 1
            self._open_serial()
            return

        if not data:
            return

        frames = self._parser.feed(data)

        for frame in frames:
            self._rx_count += 1
            self._last_frame_time = time.monotonic()

            # flag_stop: 0x00 = motors enabled
            self._motor_enabled = frame["flag_stop"] == 0x00

            # Publish odometry
            if self._publish_odom:
                self._publish_odom_from_frame(frame)

            # Publish battery
            self._publish_battery(frame)

            # Throttled status log (once per 200 frames ≈ 2 s at 100 Hz)
            self._tick += 1
            if self._tick % 200 == 1:
                self.get_logger().info(
                    f"vx={frame['vx_mps']:+.3f} m/s  "
                    f"wz={frame['serial_wz_radps']:+.3f} rad/s  "
                    f"bat={frame['voltage']:.2f} V  "
                    f"motor={'ON' if self._motor_enabled else 'OFF'}"
                )

    # ==================================================================
    # Odometry
    # ==================================================================

    def _publish_odom_from_frame(self, frame: dict) -> None:
        """Integrate velocities and publish Odometry (and optional TF)."""
        now = time.monotonic()

        vx = frame["vx_mps"]  # body-frame forward speed (m/s)
        # Apply yaw_sign to convert serial angular convention → ROS convention
        wz = self._yaw_sign * frame["serial_wz_radps"]  # rad/s in ROS convention

        # Skip integration on the very first frame (no dt available)
        if self._prev_time is None:
            self._prev_time = now
            return

        dt = now - self._prev_time
        self._prev_time = now

        # Sanity clamp for dt (avoid huge jumps after pauses)
        if dt <= 0.0 or dt > 1.0:
            return

        # Simple 2D Ackermann odometry integration (vy ≈ 0)
        self._odom_yaw += wz * dt
        # Normalise yaw to [-π, π]
        self._odom_yaw = math.atan2(math.sin(self._odom_yaw), math.cos(self._odom_yaw))

        self._odom_x += vx * math.cos(self._odom_yaw) * dt
        self._odom_y += vx * math.sin(self._odom_yaw) * dt

        # Quaternion from yaw
        half_yaw = self._odom_yaw * 0.5
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        stamp = self.get_clock().now().to_msg()

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._base_frame_id

        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (row-major 6×6, diagonal only)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        # Twist covariance
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.01  # wz

        self._odom_pub.publish(odom)

        # --- Optional TF ---
        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._frame_id
            tf.child_frame_id = self._base_frame_id
            tf.transform.translation.x = self._odom_x
            tf.transform.translation.y = self._odom_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)

    # ==================================================================
    # Battery
    # ==================================================================

    def _publish_battery(self, frame: dict) -> None:
        """Publish BatteryState from a parsed uplink frame."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = frame["voltage"]  # V
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self._battery_pub.publish(msg)
        self._last_voltage = frame["voltage"]

    # ==================================================================
    # Downlink — cmd_vel → serial
    # ==================================================================

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Subscriber callback for /cmd_vel."""
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _send_cmd_cb(self) -> None:
        """Timer callback: send a downlink velocity frame."""
        if self._ser is None or not self._ser.is_open:
            return

        now = time.monotonic()

        # Determine which Twist to send
        if (
            self._last_cmd is not None
            and (now - self._last_cmd_time) < self._command_timeout
        ):
            cmd = self._last_cmd
        else:
            # Timeout or no command yet — send zero velocity
            cmd = Twist()

        # Clamp ROS inputs
        linear_x = max(-self._max_linear_speed,
                       min(self._max_linear_speed, cmd.linear.x))
        angular_z = max(-self._max_angular_speed,
                        min(self._max_angular_speed, cmd.angular.z))

        # Convert to protocol units
        #   yaw_sign maps ROS angular convention → chassis serial convention:
        #     serial_wz = yaw_sign * ros_wz
        x_mm_s = int(linear_x * 1000)
        y_mm_s = 0  # Ackermann: no lateral motion
        z_mrad_s = int(self._yaw_sign * angular_z * 1000)

        frame = build_downlink_frame(x_mm_s, y_mm_s, z_mrad_s)

        try:
            written = self._ser.write(frame)
            if written == DOWNLINK_FRAME_LEN:
                self._tx_count += 1
            else:
                self.get_logger().warn(
                    f"Serial write incomplete: {written}/{DOWNLINK_FRAME_LEN} bytes"
                )
        except Exception as exc:
            self.get_logger().warn(f"Serial write error: {exc}")

    # ==================================================================
    # Diagnostics
    # ==================================================================

    def _publish_diagnostics_cb(self) -> None:
        """Publish diagnostic information at ~1 Hz."""
        now = time.monotonic()

        # Serial connection status
        connected = self._ser is not None and self._ser.is_open

        age_str = "N/A"
        if self._last_frame_time > 0.0:
            age = now - self._last_frame_time
            age_str = f"{age:.1f}s"

        status = DiagnosticStatus()
        status.name = "ackermann_serial_bridge"
        status.hardware_id = self._port

        if connected and self._last_frame_time > 0.0:
            status.level = DiagnosticStatus.OK
            status.message = "OK"
        elif connected:
            status.level = DiagnosticStatus.WARN
            status.message = "Connected, no data received yet"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Serial port not open"

        status.values = [
            KeyValue(key="serial_connected", value=str(connected)),
            KeyValue(key="port", value=self._port),
            KeyValue(key="rx_frames", value=str(self._rx_count)),
            KeyValue(key="tx_frames", value=str(self._tx_count)),
            KeyValue(key="checksum_errors", value=str(self._checksum_errors)),
            KeyValue(key="parse_errors", value=str(self._parse_errors)),
            KeyValue(key="motor_enabled", value=str(self._motor_enabled)),
            KeyValue(key="battery_voltage_V", value=f"{self._last_voltage:.2f}"),
            KeyValue(key="last_frame_age", value=age_str),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    # ==================================================================
    # Lifecycle
    # ==================================================================

    def destroy_node(self) -> None:
        """Send zero-velocity commands before shutting down."""
        self.get_logger().info("Shutting down — sending zero-velocity commands...")
        for _ in range(3):
            frame = build_downlink_frame(0, 0, 0)
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(frame)
                except Exception:
                    pass
            time.sleep(0.05)

        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass

        super().destroy_node()


# ======================================================================
# main
# ======================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannSerialBridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
