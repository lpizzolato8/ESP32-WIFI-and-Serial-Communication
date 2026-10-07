"""Host tests for stm32/src/pose_rx.c, fed with frames from pose_frame.py.

Run from the repo root:  python3 -m pytest stm32/test
"""
import ctypes
import os
import struct
import subprocess
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
from pose_frame import PoseFrame, FRAME_SIZE, POS_SCALE, crc16_ccitt  # noqa: E402

REV_FRAME_SIZE = 1034
STATUS_FMT = "<4s6IB3xI21s"


@pytest.fixture
def rx(tmp_path):
    """A freshly built copy of pose_rx.c (its state is file-static)."""
    so = tmp_path / "libpose_rx.so"
    subprocess.check_call([
        "gcc", "-shared", "-fPIC", "-O2", "-Wall", "-Werror",
        "-I", os.path.join(ROOT, "main"),
        os.path.join(ROOT, "stm32", "src", "pose_rx.c"), "-o", str(so)])
    lib = ctypes.CDLL(str(so))
    lib.pose_rx_feed.argtypes = [ctypes.c_uint8, ctypes.c_uint32]
    lib.pose_rx_tick.argtypes = [ctypes.c_uint32]
    lib.pose_rx_build_status.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    return lib


def feed(lib, data: bytes, now: int = 0):
    for b in data:
        lib.pose_rx_feed(b, now)


def status(lib, now: int = 0, counter: int = 0) -> dict:
    buf = ctypes.create_string_buffer(REV_FRAME_SIZE)
    lib.pose_rx_build_status(buf, counter, now)
    raw = buf.raw
    assert raw[:4] == b"REV\xAA"
    assert struct.unpack_from("<I", raw, 4)[0] == counter
    assert struct.unpack_from("<H", raw, REV_FRAME_SIZE - 2)[0] == crc16_ccitt(raw[:-2])
    fields = struct.unpack_from(STATUS_FMT, raw, 8)
    keys = ("tag uptime_ms frames_ok crc_fail bytes_skipped uart_errors "
            "failsafe_count failsafe last_rx_age_ms last_pose").split()
    return dict(zip(keys, fields))


def frame(seq: int) -> bytes:
    return PoseFrame.pack(seq / POS_SCALE, -1.5, 0.25, 1.0, 0.0, 0.0, 0.0)


def test_clean_stream(rx):
    feed(rx, b"".join(frame(i) for i in range(100)))
    s = status(rx)
    assert s["tag"] == b"STM1"
    assert (s["frames_ok"], s["crc_fail"], s["bytes_skipped"]) == (100, 0, 0)
    assert s["last_pose"] == frame(99)


def test_garbage_before_first_frame(rx):
    feed(rx, b"\x00\xff\x12\xab" + frame(0))
    s = status(rx)
    assert s["frames_ok"] == 1
    assert s["bytes_skipped"] == 4


def test_corrupt_frame_is_dropped(rx):
    bad = bytearray(frame(1))
    bad[7] ^= 0x01
    feed(rx, frame(0) + bytes(bad) + frame(2))
    s = status(rx)
    assert (s["frames_ok"], s["crc_fail"]) == (2, 1)
    assert s["last_pose"] == frame(2)


def test_resync_into_frame_inside_false_start(rx):
    # A stray magic followed by a real frame 3 bytes later: the parser must
    # re-scan the bytes after the false start and still find the real one.
    feed(rx, b"\xcd\xab\x00" + frame(5) + frame(6))
    s = status(rx)
    assert s["frames_ok"] == 2
    assert s["crc_fail"] == 1
    assert s["last_pose"] == frame(6)


def test_repeated_first_magic_byte(rx):
    feed(rx, b"\xcd" + frame(0))          # CD CD AB ...
    s = status(rx)
    assert (s["frames_ok"], s["bytes_skipped"]) == (1, 1)


def test_byte_at_a_time_split(rx):
    data = frame(0) + frame(1)
    for i, b in enumerate(data):
        rx.pose_rx_feed(b, i)
    assert status(rx)["frames_ok"] == 2


def test_failsafe(rx):
    assert rx.pose_rx_tick(0) == 1               # nothing received yet
    assert status(rx)["last_rx_age_ms"] == 0xFFFFFFFF
    feed(rx, frame(0), now=1000)
    assert rx.pose_rx_tick(1050) == 0
    assert rx.pose_rx_tick(1100) == 0            # exactly at the timeout
    assert rx.pose_rx_tick(1101) == 1
    assert rx.pose_rx_tick(1200) == 1
    s = status(rx, now=1200)
    assert (s["failsafe"], s["failsafe_count"], s["last_rx_age_ms"]) == (1, 1, 200)
    feed(rx, frame(1), now=1300)
    assert rx.pose_rx_tick(1300) == 0
    assert rx.pose_rx_tick(1500) == 1
    assert status(rx)["failsafe_count"] == 2


def test_status_payload_is_zero_padded(rx):
    buf = ctypes.create_string_buffer(REV_FRAME_SIZE)
    rx.pose_rx_build_status(buf, 7, 0)
    payload = buf.raw[8:8 + 1024]
    assert payload[struct.calcsize(STATUS_FMT):] == bytes(1024 - struct.calcsize(STATUS_FMT))
