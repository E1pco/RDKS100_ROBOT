"""
Ackermann chassis serial protocol implementation.

Pure protocol functions with no ROS2 dependency — can be unit-tested standalone.

Uplink frame (chassis → host): 24 bytes
  [0]     0x7B header
  [1]     flag_stop  (0x00 = motors enabled)
  [2:4]   x_mm_s     (int16 BE, mm/s)
  [4:6]   y_mm_s     (int16 BE, mm/s)
  [6:8]   z_mrad_s   (int16 BE, angular vel * 1000, rad/s)
  [8:10]  acc_x_raw  (int16 BE)
  [10:12] acc_y_raw  (int16 BE)
  [12:14] acc_z_raw  (int16 BE)
  [14:16] gyro_x_raw (int16 BE)
  [16:18] gyro_y_raw (int16 BE)
  [18:20] gyro_z_raw (int16 BE)
  [20:22] battery_mv (int16 BE, mV)
  [22]    BCC (XOR of bytes 0..21)
  [23]    0x7D tail

Downlink frame (host → chassis): 11 bytes
  [0]     0x7B header
  [1]     reserved 0x00
  [2]     reserved 0x00
  [3:5]   x_mm_s     (int16 BE, mm/s)
  [5:7]   y_mm_s     (int16 BE, mm/s — always 0 for Ackermann)
  [7:9]   z_mrad_s   (int16 BE, angular vel * 1000, rad/s)
  [9]     BCC (XOR of bytes 0..8)
  [10]    0x7D tail
"""

from __future__ import annotations

import struct
from typing import List

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
HEADER = 0x7B
TAIL = 0x7D
UPLINK_FRAME_LEN = 24
DOWNLINK_FRAME_LEN = 11
INT16_MAX = 32767
INT16_MIN = -32768

# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def calc_bcc(data: bytes) -> int:
    """Return XOR checksum over *data*."""
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc & 0xFF


def clamp_int16(value: int) -> int:
    """Clamp *value* to signed 16-bit range."""
    if value > INT16_MAX:
        return INT16_MAX
    if value < INT16_MIN:
        return INT16_MIN
    return value


def pack_int16_be(value: int) -> bytes:
    """Pack a signed 16-bit integer as big-endian two bytes."""
    return struct.pack(">h", clamp_int16(value))


def unpack_int16_be(high: int, low: int) -> int:
    """Unpack two bytes (high, low) as a signed big-endian 16-bit integer."""
    return struct.unpack(">h", bytes([high & 0xFF, low & 0xFF]))[0]


# ---------------------------------------------------------------------------
# Downlink frame builder
# ---------------------------------------------------------------------------

def build_downlink_frame(x_mm_s: int, y_mm_s: int, z_mrad_s: int) -> bytes:
    """Build an 11-byte downlink velocity command frame.

    Parameters
    ----------
    x_mm_s : int
        Target forward speed in mm/s (will be clamped to int16).
    y_mm_s : int
        Target lateral speed in mm/s.  Ackermann vehicles should pass 0.
    z_mrad_s : int
        Target angular velocity in mrad/s (rad/s × 1000, will be clamped).
    """
    x_bytes = pack_int16_be(x_mm_s)
    y_bytes = pack_int16_be(y_mm_s)
    z_bytes = pack_int16_be(z_mrad_s)

    # Header + 2 reserved bytes + 3 × int16 payload
    payload = bytes([HEADER, 0x00, 0x00]) + x_bytes + y_bytes + z_bytes
    bcc = calc_bcc(payload)
    return payload + bytes([bcc, TAIL])


# ---------------------------------------------------------------------------
# Uplink single-frame parser
# ---------------------------------------------------------------------------

def parse_uplink_frame(frame: bytes) -> dict:
    """Parse a raw 24-byte uplink frame and return a dict of fields.

    The caller is responsible for ensuring *frame* has the correct length.
    ``checksum_ok`` is set to ``True`` only when header, tail, and BCC all
    match.
    """
    result: dict = {}

    # Header / tail check
    if len(frame) != UPLINK_FRAME_LEN:
        result["checksum_ok"] = False
        return result

    result["header_ok"] = frame[0] == HEADER
    result["tail_ok"] = frame[UPLINK_FRAME_LEN - 1] == TAIL
    result["checksum_ok"] = (
        result["header_ok"]
        and result["tail_ok"]
        and (calc_bcc(frame[:22]) == frame[22])
    )

    # flag_stop
    result["flag_stop"] = frame[1]

    # Velocities (mm/s → m/s)
    result["x_mm_s"] = unpack_int16_be(frame[2], frame[3])
    result["y_mm_s"] = unpack_int16_be(frame[4], frame[5])
    result["z_mrad_s"] = unpack_int16_be(frame[6], frame[7])
    result["vx_mps"] = result["x_mm_s"] / 1000.0
    result["vy_mps"] = result["y_mm_s"] / 1000.0
    # serial_wz_radps: raw angular velocity from the chassis (rad/s)
    result["serial_wz_radps"] = result["z_mrad_s"] / 1000.0

    # IMU raw values — parsed but NOT published
    result["acc_x_raw"] = unpack_int16_be(frame[8], frame[9])
    result["acc_y_raw"] = unpack_int16_be(frame[10], frame[11])
    result["acc_z_raw"] = unpack_int16_be(frame[12], frame[13])
    result["gyro_x_raw"] = unpack_int16_be(frame[14], frame[15])
    result["gyro_y_raw"] = unpack_int16_be(frame[16], frame[17])
    result["gyro_z_raw"] = unpack_int16_be(frame[18], frame[19])

    # Battery
    result["battery_mv"] = unpack_int16_be(frame[20], frame[21])
    result["voltage"] = result["battery_mv"] / 1000.0

    return result


# ---------------------------------------------------------------------------
# Streaming uplink frame parser (handles half / multi / corrupt frames)
# ---------------------------------------------------------------------------

class UplinkFrameParser:
    """Stateful streaming parser for uplink serial data.

    Feed raw bytes via :meth:`feed`; it returns a list of successfully
    parsed frame dicts (may be empty on partial / corrupt data).  Internally
    maintains a ``bytearray`` buffer and automatically discards leading junk
    when re-synchronising to the next ``0x7B`` header.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[dict]:
        """Append *data* to the internal buffer and extract all complete,
        valid uplink frames.

        Returns
        -------
        list[dict]
            One dict per successfully parsed frame (may be empty).
        """
        self._buf.extend(data)
        frames: List[dict] = []

        while True:
            # Need at least enough bytes for one frame
            if len(self._buf) < UPLINK_FRAME_LEN:
                break

            # Find the next header byte
            try:
                hdr_idx = self._buf.index(HEADER)
            except ValueError:
                # No header in buffer — discard everything
                self._buf.clear()
                break

            # Discard leading junk before the header
            if hdr_idx > 0:
                del self._buf[:hdr_idx]

            # Still not enough bytes for a full frame?
            if len(self._buf) < UPLINK_FRAME_LEN:
                break

            # Quick tail check before full parse
            if self._buf[UPLINK_FRAME_LEN - 1] != TAIL:
                # Bad tail — skip this header and try to resync
                del self._buf[0]
                continue

            # Extract the candidate frame bytes
            candidate = bytes(self._buf[:UPLINK_FRAME_LEN])
            parsed = parse_uplink_frame(candidate)

            if parsed.get("checksum_ok"):
                frames.append(parsed)
                # Consume the frame from the buffer
                del self._buf[:UPLINK_FRAME_LEN]
            else:
                # BCC mismatch — skip this header byte and resync
                del self._buf[0]

        return frames

    def reset(self) -> None:
        """Clear the internal buffer."""
        self._buf.clear()
if __name__ == "__main__":
    import serial
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    # Build a downlink frame
    downlink = build_downlink_frame(x_mm_s=100, y_mm_s=0, z_mrad_s=-200)
    print("Downlink frame:", downlink.hex())
    ser.write(downlink)



