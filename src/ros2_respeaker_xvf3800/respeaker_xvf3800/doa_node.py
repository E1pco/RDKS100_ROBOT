import struct
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt16
import usb.core
import usb.util


CONTROL_SUCCESS = 0
SERVICER_COMMAND_RETRY = 64


class ReSpeakerXVF3800:
    TIMEOUT_MS = 100000
    DOA_RESID = 20
    DOA_CMDID = 18
    DOA_VALUE_COUNT = 2

    def __init__(self, vid: int, pid: int):
        self._device = usb.core.find(idVendor=vid, idProduct=pid)
        if self._device is None:
            raise RuntimeError(f"reSpeaker XVF3800 not found: vid=0x{vid:04x}, pid=0x{pid:04x}")

    def read_doa(self) -> Tuple[int, bool]:
        response = self._read_uint16(self.DOA_RESID, self.DOA_CMDID, self.DOA_VALUE_COUNT)
        return response[0], bool(response[1])

    def close(self) -> None:
        usb.util.dispose_resources(self._device)

    def _read_uint16(self, resid: int, cmdid: int, value_count: int) -> Tuple[int, ...]:
        request = 0x80 | cmdid
        length = value_count * 2 + 1

        for _ in range(100):
            response = self._device.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0,
                request,
                resid,
                length,
                self.TIMEOUT_MS,
            )

            status = response[0]
            if status == CONTROL_SUCCESS:
                return struct.unpack("<" + "H" * value_count, response.tobytes()[1:])
            if status != SERVICER_COMMAND_RETRY:
                raise RuntimeError(f"device returned status code {status}")

        raise TimeoutError("device kept returning retry status while reading DOA_VALUE")


class DoaNode(Node):
    def __init__(self):
        super().__init__("respeaker_xvf3800_doa")

        self.declare_parameter("vid", 0x2886)
        self.declare_parameter("pid", 0x001A)
        self.declare_parameter("poll_rate_hz", 10.0)

        self._device: Optional[ReSpeakerXVF3800] = None
        self._doa_pub = self.create_publisher(UInt16, "doa", 10)
        self._speech_pub = self.create_publisher(Bool, "speech_detected", 10)

        vid = self.get_parameter("vid").get_parameter_value().integer_value
        pid = self.get_parameter("pid").get_parameter_value().integer_value
        poll_rate_hz = self.get_parameter("poll_rate_hz").get_parameter_value().double_value

        self._device = ReSpeakerXVF3800(vid, pid)
        self.get_logger().info(f"Connected to reSpeaker XVF3800 USB device 0x{vid:04x}:0x{pid:04x}")

        period = 1.0 / max(poll_rate_hz, 0.1)
        self._timer = self.create_timer(period, self._poll)

    def destroy_node(self):
        if self._device is not None:
            self._device.close()
        super().destroy_node()

    def _poll(self) -> None:
        if self._device is None:
            return

        try:
            doa, speech_detected = self._device.read_doa()
        except Exception as exc:
            self.get_logger().error(f"Failed to read DOA_VALUE: {exc}")
            return

        doa_msg = UInt16()
        doa_msg.data = doa
        self._doa_pub.publish(doa_msg)

        speech_msg = Bool()
        speech_msg.data = speech_detected
        self._speech_pub.publish(speech_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DoaNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
