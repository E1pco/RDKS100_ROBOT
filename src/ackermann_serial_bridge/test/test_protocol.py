"""Unit tests for ackermann_serial_bridge.protocol (no ROS2 required)."""

import struct
import pytest

from ackermann_serial_bridge.protocol import (
    HEADER,
    TAIL,
    UPLINK_FRAME_LEN,
    DOWNLINK_FRAME_LEN,
    calc_bcc,
    clamp_int16,
    pack_int16_be,
    unpack_int16_be,
    build_downlink_frame,
    parse_uplink_frame,
    UplinkFrameParser,
)


# =========================================================================
# Helpers
# =========================================================================

# Canonical 24-byte uplink test frame from the specification
UPLINK_TEST_HEX = "7B 00 00 9B 00 00 FF DF 00 60 00 0C 40 A8 FF FD 00 06 00 1E 5B 87 82 7D"
UPLINK_TEST_BYTES = bytes.fromhex(UPLINK_TEST_HEX)


# =========================================================================
# calc_bcc
# =========================================================================

class TestCalcBCC:
    def test_downlink_100mm_forward(self):
        """BCC of a 100 mm/s forward downlink frame = 0x1F."""
        # bytes 0-8 of the expected frame: 7B 00 00 00 64 00 00 00 00
        payload = bytes([0x7B, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00])
        assert calc_bcc(payload) == 0x1F

    def test_zero_length(self):
        assert calc_bcc(b"") == 0x00

    def test_single_byte(self):
        assert calc_bcc(b"\xAB") == 0xAB


# =========================================================================
# clamp_int16
# =========================================================================

class TestClampInt16:
    def test_within_range(self):
        assert clamp_int16(100) == 100
        assert clamp_int16(-100) == -100

    def test_upper_clamp(self):
        assert clamp_int16(40000) == 32767
        assert clamp_int16(32767) == 32767

    def test_lower_clamp(self):
        assert clamp_int16(-40000) == -32768
        assert clamp_int16(-32768) == -32768

    def test_zero(self):
        assert clamp_int16(0) == 0


# =========================================================================
# pack_int16_be / unpack_int16_be
# =========================================================================

class TestInt16:
    def test_pack_positive(self):
        assert pack_int16_be(100) == b"\x00\x64"

    def test_pack_negative(self):
        assert pack_int16_be(-33) == b"\xFF\xDF"

    def test_pack_zero(self):
        assert pack_int16_be(0) == b"\x00\x00"

    def test_roundtrip(self):
        for v in [0, 1, -1, 100, -100, 32767, -32768]:
            data = pack_int16_be(v)
            assert unpack_int16_be(data[0], data[1]) == v

    def test_unpack_known(self):
        # 0x009B = 155
        assert unpack_int16_be(0x00, 0x9B) == 155
        # 0xFFDF = -33
        assert unpack_int16_be(0xFF, 0xDF) == -33
        # 0x40A8 = 16552
        assert unpack_int16_be(0x40, 0xA8) == 16552


# =========================================================================
# build_downlink_frame
# =========================================================================

class TestBuildDownlinkFrame:
    def test_100mm_forward(self):
        """100 mm/s forward, 0 lateral, 0 angular → exact byte match."""
        expected = bytes.fromhex("7B 00 00 00 64 00 00 00 00 1F 7D")
        frame = build_downlink_frame(100, 0, 0)
        assert frame == expected

    def test_length(self):
        frame = build_downlink_frame(0, 0, 0)
        assert len(frame) == DOWNLINK_FRAME_LEN

    def test_header_tail(self):
        frame = build_downlink_frame(500, 0, -200)
        assert frame[0] == HEADER
        assert frame[-1] == TAIL

    def test_bcc_valid(self):
        frame = build_downlink_frame(250, 0, -500)
        assert calc_bcc(frame[:9]) == frame[9]

    def test_zero_velocity(self):
        frame = build_downlink_frame(0, 0, 0)
        expected = bytes.fromhex("7B 00 00 00 00 00 00 00 00 7B 7D")
        assert frame == expected

    def test_y_always_zero_for_ackermann(self):
        """Ackermann: y_mm_s is hardcoded to 0 in the frame."""
        frame = build_downlink_frame(100, 0, 500)
        assert frame[5] == 0x00 and frame[6] == 0x00


# =========================================================================
# parse_uplink_frame
# =========================================================================

class TestParseUplinkFrame:
    def test_spec_frame(self):
        """Parse the test frame from the specification and check all fields."""
        result = parse_uplink_frame(UPLINK_TEST_BYTES)

        assert result["checksum_ok"] is True
        assert result["header_ok"] is True
        assert result["tail_ok"] is True

        assert result["flag_stop"] == 0

        # Velocities
        assert result["x_mm_s"] == 155
        assert result["y_mm_s"] == 0
        assert result["z_mrad_s"] == -33

        assert abs(result["vx_mps"] - 0.155) < 1e-6
        assert abs(result["vy_mps"] - 0.0) < 1e-6
        assert abs(result["serial_wz_radps"] - (-0.033)) < 1e-6

        # IMU raw (just verify the ones specified)
        assert result["acc_z_raw"] == 16552

        # Battery
        assert result["battery_mv"] == 23431
        assert abs(result["voltage"] - 23.431) < 1e-6

    def test_wrong_header(self):
        bad = bytearray(UPLINK_TEST_BYTES)
        bad[0] = 0x00
        result = parse_uplink_frame(bytes(bad))
        assert result["checksum_ok"] is False

    def test_wrong_tail(self):
        bad = bytearray(UPLINK_TEST_BYTES)
        bad[23] = 0x00
        result = parse_uplink_frame(bytes(bad))
        assert result["checksum_ok"] is False

    def test_corrupt_bcc(self):
        bad = bytearray(UPLINK_TEST_BYTES)
        bad[22] ^= 0xFF  # flip BCC
        result = parse_uplink_frame(bytes(bad))
        assert result["checksum_ok"] is False

    def test_wrong_length(self):
        result = parse_uplink_frame(b"\x7B\x00\x7D")
        assert result["checksum_ok"] is False


# =========================================================================
# UplinkFrameParser — streaming
# =========================================================================

class TestUplinkFrameParser:
    def test_single_frame(self):
        parser = UplinkFrameParser()
        frames = parser.feed(UPLINK_TEST_BYTES)
        assert len(frames) == 1
        assert frames[0]["x_mm_s"] == 155
        assert frames[0]["checksum_ok"] is True

    def test_two_frames_concatenated(self):
        """Two complete frames fed at once → two results."""
        parser = UplinkFrameParser()
        data = UPLINK_TEST_BYTES + UPLINK_TEST_BYTES
        frames = parser.feed(data)
        assert len(frames) == 2
        assert frames[0]["x_mm_s"] == 155
        assert frames[1]["x_mm_s"] == 155

    def test_half_frame_then_complete(self):
        """Split a frame in two; first feed → 0 results, second → 1 result."""
        parser = UplinkFrameParser()
        part1 = UPLINK_TEST_BYTES[:12]
        part2 = UPLINK_TEST_BYTES[12:]

        frames1 = parser.feed(part1)
        assert len(frames1) == 0

        frames2 = parser.feed(part2)
        assert len(frames2) == 1
        assert frames2[0]["x_mm_s"] == 155

    def test_junk_before_header(self):
        """Leading junk bytes are discarded; valid frame is still found."""
        parser = UplinkFrameParser()
        junk = b"\x00\xFF\xAA\xBB"
        frames = parser.feed(junk + UPLINK_TEST_BYTES)
        assert len(frames) == 1
        assert frames[0]["checksum_ok"] is True

    def test_corrupt_frame_recovery(self):
        """A corrupt frame is skipped; a subsequent valid frame is parsed."""
        parser = UplinkFrameParser()
        bad = bytearray(UPLINK_TEST_BYTES)
        bad[22] ^= 0xFF  # corrupt BCC
        frames = parser.feed(bytes(bad) + UPLINK_TEST_BYTES)
        # The bad frame may or may not produce a result (checksum_ok=False),
        # but the second good frame must be found.
        good = [f for f in frames if f["checksum_ok"]]
        assert len(good) >= 1
        assert good[-1]["x_mm_s"] == 155

    def test_empty_feed(self):
        parser = UplinkFrameParser()
        assert parser.feed(b"") == []

    def test_reset(self):
        parser = UplinkFrameParser()
        parser.feed(UPLINK_TEST_BYTES[:12])
        parser.reset()
        # After reset, feeding the second half alone should NOT yield a frame
        frames = parser.feed(UPLINK_TEST_BYTES[12:])
        assert len(frames) == 0

    def test_byte_by_byte(self):
        """Feed one byte at a time — parser must eventually return the frame."""
        parser = UplinkFrameParser()
        result = None
        for i in range(len(UPLINK_TEST_BYTES)):
            frames = parser.feed(bytes([UPLINK_TEST_BYTES[i]]))
            if frames:
                result = frames[0]
        assert result is not None
        assert result["x_mm_s"] == 155
        assert result["checksum_ok"] is True
