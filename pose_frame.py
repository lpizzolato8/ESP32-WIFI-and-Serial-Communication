#!/usr/bin/env python3
"""
pose_frame.py  –  Python codec for the binary pose frame.

Frame layout (20 bytes, little-endian):
  [0]      header   uint8   magic 0xA5
  [1:4]    pos_x    int24   metres × 10000  (0.1 mm / LSB, ±99.9 m)
  [4:7]    pos_y    int24
  [7:10]   pos_z    int24
  [10:12]  quat_w   int16   component × 32767  (≈0.003° / LSB)
  [12:14]  quat_x   int16
  [14:16]  quat_y   int16
  [16:18]  quat_z   int16
  [18:20]  crc16    uint16  CRC-16/CCITT-FALSE over bytes [0..17]

Usage:
    from pose_frame import PoseFrame

    # Encode
    raw = PoseFrame.pack(x=1.234, y=0.0, z=-0.500,
                         w=1.0, qx=0.0, qy=0.0, qz=0.0)

    # Decode
    frame = PoseFrame.unpack(raw)
    print(frame)
"""

import struct
from dataclasses import dataclass

# ── Constants ────────────────────────────────────────────────────────────────

FRAME_HEADER  = 0xA5
FRAME_SIZE    = 20
POS_SCALE     = 10000.0   # raw = metres × 10000
QUAT_SCALE    = 32767.0   # raw = component × 32767


# ── CRC-16/CCITT-FALSE ───────────────────────────────────────────────────────

def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE  poly=0x1021  init=0xFFFF  refin=False  refout=False"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# ── int24 helpers ────────────────────────────────────────────────────────────

def _pack_int24(v: int) -> bytes:
    """Signed int32 → 3 bytes little-endian."""
    v &= 0xFFFFFF
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])


def _unpack_int24(b: bytes) -> int:
    """3 bytes little-endian → signed int32."""
    v = b[0] | (b[1] << 8) | (b[2] << 16)
    if v & 0x800000:
        v -= 0x1000000
    return v


# ── Frame dataclass ──────────────────────────────────────────────────────────

@dataclass
class PoseFrame:
    x:  float   # metres
    y:  float
    z:  float
    w:  float   # quaternion (unit)
    qx: float
    qy: float
    qz: float

    # ── Encode ───────────────────────────────────────────────────────────────

    def to_bytes(self) -> bytes:
        payload = bytes([FRAME_HEADER])
        payload += _pack_int24(round(self.x  * POS_SCALE))
        payload += _pack_int24(round(self.y  * POS_SCALE))
        payload += _pack_int24(round(self.z  * POS_SCALE))
        payload += struct.pack("<hhhh",
                               round(self.w  * QUAT_SCALE),
                               round(self.qx * QUAT_SCALE),
                               round(self.qy * QUAT_SCALE),
                               round(self.qz * QUAT_SCALE))
        crc = crc16_ccitt(payload)           # CRC over bytes [0..17]
        payload += struct.pack("<H", crc)    # append 2-byte LE CRC
        assert len(payload) == FRAME_SIZE
        return payload

    @staticmethod
    def pack(x: float, y: float, z: float,
             w: float, qx: float, qy: float, qz: float) -> bytes:
        """Convenience wrapper — returns raw bytes ready to send over UDP."""
        return PoseFrame(x, y, z, w, qx, qy, qz).to_bytes()

    # ── Decode ───────────────────────────────────────────────────────────────

    @staticmethod
    def unpack(data: bytes) -> "PoseFrame":
        """
        Parse raw bytes into a PoseFrame.
        Raises ValueError on bad header or CRC mismatch.
        """
        if len(data) < FRAME_SIZE:
            raise ValueError(f"Frame too short: {len(data)} < {FRAME_SIZE}")

        if data[0] != FRAME_HEADER:
            raise ValueError(f"Bad header: 0x{data[0]:02X} (expected 0x{FRAME_HEADER:02X})")

        expected_crc = crc16_ccitt(data[:FRAME_SIZE - 2])
        received_crc = struct.unpack_from("<H", data, FRAME_SIZE - 2)[0]
        if expected_crc != received_crc:
            raise ValueError(f"CRC mismatch: got 0x{received_crc:04X}, expected 0x{expected_crc:04X}")

        x  = _unpack_int24(data[1:4])  / POS_SCALE
        y  = _unpack_int24(data[4:7])  / POS_SCALE
        z  = _unpack_int24(data[7:10]) / POS_SCALE
        w, qx, qy, qz = struct.unpack_from("<hhhh", data, 10)
        return PoseFrame(
            x=x, y=y, z=z,
            w=w  / QUAT_SCALE,
            qx=qx / QUAT_SCALE,
            qy=qy / QUAT_SCALE,
            qz=qz / QUAT_SCALE,
        )

    def __str__(self) -> str:
        return (f"pos=({self.x:+.4f}, {self.y:+.4f}, {self.z:+.4f}) m  "
                f"quat=({self.w:+.5f}, {self.qx:+.5f}, {self.qy:+.5f}, {self.qz:+.5f})")


# ── Quick self-test ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    import math

    tests = [
        dict(x=0.0,    y=0.0,    z=0.0,    w=1.0,  qx=0.0,  qy=0.0,  qz=0.0),
        dict(x=99.9,   y=-99.9,  z=12.345, w=0.0,  qx=1.0,  qy=0.0,  qz=0.0),
        dict(x=0.0001, y=0.0,    z=0.0,    w=0.707, qx=0.707, qy=0.0, qz=0.0),
        dict(x=-50.0,  y=25.123, z=-0.001, w=0.5,  qx=0.5,  qy=0.5,  qz=0.5),
    ]

    print(f"{'Input':>45}  →  {'Decoded':>55}  {'OK':>3}")
    print("─" * 120)
    for t in tests:
        raw    = PoseFrame.pack(**t)
        decoded = PoseFrame.unpack(raw)
        ok = (abs(decoded.x  - t["x"])  < 0.00011 and
              abs(decoded.y  - t["y"])  < 0.00011 and
              abs(decoded.z  - t["z"])  < 0.00011 and
              abs(decoded.w  - t["w"])  < 0.000035 and
              abs(decoded.qx - t["qx"]) < 0.000035 and
              abs(decoded.qy - t["qy"]) < 0.000035 and
              abs(decoded.qz - t["qz"]) < 0.000035)
        print(f"  {str(PoseFrame(**t)):55}  →  {str(decoded):55}  {'✓' if ok else '✗ FAIL'}")
        assert ok, f"Precision failure on {t}"

    print(f"\nAll tests passed. Frame size = {FRAME_SIZE} bytes.")