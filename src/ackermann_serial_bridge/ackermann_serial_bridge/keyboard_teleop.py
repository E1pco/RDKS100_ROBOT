"""
ROS2 node: KeyboardTeleop

Publishes Twist commands on /cmd_vel via keyboard input.
Designed for Ackermann vehicles (linear.x = forward speed, angular.z = steering).

Controls (single key, no Enter needed in raw mode):
    W / ↑    accelerate forward
    S / ↓    accelerate backward
    A / ←    steer left
    D / →    steer right
    SPACE    emergency stop (zero velocity)
    Q / ESC  quit

When stdin is not a TTY (e.g. piped), falls back to line-based input
(one character per line).

Speed is adjusted incrementally; steering is held while key is pressed.
"""

from __future__ import annotations

import math
import os
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist


# ── Key constants ────────────────────────────────────────────────────────────
KEY_W = "w"
KEY_S = "s"
KEY_A = "a"
KEY_D = "d"
KEY_SPACE = " "
KEY_Q = "q"
KEY_ESC = "\x1b"

_RAW_KEYS = {KEY_W, KEY_S, KEY_A, KEY_D, KEY_SPACE, KEY_Q}


def _is_tty() -> bool:
    return os.isatty(sys.stdin.fileno())


def _read_key_raw() -> str:
    """Read a single keypress from stdin (blocking, raw mode)."""
    import tty as _tty

    fd = sys.stdin.fileno()
    old_settings = _tty.tcgetattr(fd)
    try:
        _tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                ch3 = sys.stdin.read(1)
                # Map arrow keys to WASD
                return {"A": KEY_W, "B": KEY_S, "D": KEY_A, "C": KEY_D}.get(ch3, "")
            return KEY_ESC
        return ch
    finally:
        _tty.tcsetattr(fd, _tty.TCSADRAIN, old_settings)


def _read_key_line() -> str:
    """Read one line from stdin and return the first non-empty character."""
    line = sys.stdin.readline()
    if not line:
        return KEY_Q  # EOF → quit
    return line.strip()[:1] if line.strip() else ""


# ── Help banner ──────────────────────────────────────────────────────────────

BANNER_TTY = """
╔══════════════════════════════════════════════════════════╗
║              Ackermann Keyboard Teleop                   ║
╠══════════════════════════════════════════════════════════╣
║  W / ↑    accelerate forward        D / →    steer right ║
║  S / ↓    accelerate backward       A / ←    steer left  ║
║  SPACE    emergency stop            Q / ESC  quit        ║
╠══════════════════════════════════════════════════════════╣
║  speed_step={speed_step:.2f} m/s   steer_step={steer_step:.2f} rad   ║
║  max_speed={max_speed:.2f} m/s    max_steer={max_steer:.2f} rad    ║
╚══════════════════════════════════════════════════════════╝
"""

BANNER_LINE = """
╔══════════════════════════════════════════════════════════╗
║           Ackermann Keyboard Teleop (line mode)          ║
╠══════════════════════════════════════════════════════════╣
║  w  accelerate forward              d  steer right       ║
║  s  accelerate backward             a  steer left        ║
║  x  emergency stop                  q  quit              ║
║                                                          ║
║  Type a letter + Enter for each action.                  ║
╠══════════════════════════════════════════════════════════╣
║  speed_step={speed_step:.2f} m/s   steer_step={steer_step:.2f} rad   ║
║  max_speed={max_speed:.2f} m/s    max_steer={max_steer:.2f} rad    ║
╚══════════════════════════════════════════════════════════╝
"""


class KeyboardTeleop(Node):
    """Keyboard-driven Twist publisher for Ackermann vehicles."""

    def __init__(self) -> None:
        super().__init__("keyboard_teleop")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("max_speed", 1.0)       # m/s
        self.declare_parameter("max_steer", 1.0)        # rad
        self.declare_parameter("speed_step", 0.1)       # m/s per keypress
        self.declare_parameter("steer_step", 0.2)       # rad per keypress
        self.declare_parameter("publish_rate", 20.0)    # Hz

        self._topic: str = self.get_parameter("topic").value
        self._max_speed: float = self.get_parameter("max_speed").value
        self._max_steer: float = self.get_parameter("max_steer").value
        self._speed_step: float = self.get_parameter("speed_step").value
        self._steer_step: float = self.get_parameter("steer_step").value
        self._publish_rate: float = self.get_parameter("publish_rate").value

        # ── Publisher ────────────────────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Twist, self._topic, qos)

        # ── State ────────────────────────────────────────────────────────────
        self._speed: float = 0.0
        self._steer: float = 0.0
        self._running: bool = True
        self._use_raw: bool = _is_tty()

        # ── Timer: publish at fixed rate ─────────────────────────────────────
        period = 1.0 / max(self._publish_rate, 1.0)
        self._timer = self.create_timer(period, self._publish_cb)

        # ── Keyboard thread ─────────────────────────────────────────────────
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        # Print banner
        banner = BANNER_TTY if self._use_raw else BANNER_LINE
        print(banner.format(
            speed_step=self._speed_step,
            steer_step=self._steer_step,
            max_speed=self._max_speed,
            max_steer=self._max_steer,
        ))
        if self._use_raw:
            self._print_status()
        else:
            print("w/s/a/d/x/q > ", end="", flush=True)

    # ==================================================================
    # Key handling
    # ==================================================================

    def _key_loop(self) -> None:
        """Read keys in a background thread and update speed/steer."""
        read_fn = _read_key_raw if self._use_raw else _read_key_line

        while self._running and rclpy.ok():
            try:
                key = read_fn()
            except Exception:
                break

            if not key:
                continue

            if key in (KEY_Q, KEY_ESC):
                self.get_logger().info("Quit requested.")
                # Send zero velocity before shutdown
                try:
                    self._pub.publish(Twist())
                except Exception:
                    pass
                self._running = False
                rclpy.try_shutdown()
                break

            elif key == KEY_W:
                self._speed = min(self._speed + self._speed_step, self._max_speed)

            elif key == KEY_S:
                self._speed = max(self._speed - self._speed_step, -self._max_speed)

            elif key == KEY_A:
                self._steer = min(self._steer + self._steer_step, self._max_steer)

            elif key == KEY_D:
                self._steer = max(self._steer - self._steer_step, -self._max_steer)

            elif key in (KEY_SPACE, "x"):
                self._speed = 0.0
                self._steer = 0.0

            if self._use_raw:
                self._print_status()
            else:
                print(
                    f"  speed={self._speed:+.2f} m/s  "
                    f"steer={self._steer:+.2f} rad\n"
                    f"w/s/a/d/x/q > ",
                    end="",
                    flush=True,
                )

    # ==================================================================
    # Publish
    # ==================================================================

    def _publish_cb(self) -> None:
        """Timer callback: publish current Twist."""
        msg = Twist()
        msg.linear.x = self._speed
        msg.angular.z = self._steer
        self._pub.publish(msg)

    # ==================================================================
    # Display
    # ==================================================================

    def _print_status(self) -> None:
        """Print current speed and steer to terminal."""
        bar_width = 20

        # Speed bar
        speed_ratio = self._speed / self._max_speed if self._max_speed > 0 else 0
        speed_pos = int((speed_ratio + 1.0) / 2.0 * bar_width)
        speed_bar = [" "] * (bar_width + 1)
        speed_bar[max(0, min(bar_width, speed_pos))] = "█"
        speed_str = "".join(speed_bar)

        # Steer bar
        steer_ratio = self._steer / self._max_steer if self._max_steer > 0 else 0
        steer_pos = int((steer_ratio + 1.0) / 2.0 * bar_width)
        steer_bar = [" "] * (bar_width + 1)
        steer_bar[max(0, min(bar_width, steer_pos))] = "█"
        steer_str = "".join(steer_bar)

        print(
            f"\r\033[K"
            f"speed: [{speed_str}] {self._speed:+.2f} m/s  "
            f"steer: [{steer_str}] {self._steer:+.2f} rad",
            end="",
            flush=True,
        )

    # ==================================================================
    # Lifecycle
    # ==================================================================

    def destroy_node(self) -> None:
        self._running = False
        # Send zero velocity on exit (ignore if context already invalid)
        try:
            msg = Twist()
            self._pub.publish(msg)
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print()  # newline after status bar


if __name__ == "__main__":
    main()
