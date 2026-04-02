import struct

MAGIC      = 0xABCD
FRAME_SIZE = 21       # 2 magic + 9 pos (3×int24) + 8 quat (4×int16) + 2 CRC

POS_SCALE  = 10000.0  # metres → int24   (0.1 mm resolution, ±838 m range)
QUAT_SCALE = 32767.0  # [-1, 1] → int16  (~0.00003 resolution)

MAGIC_BYTES = struct.pack("<H", MAGIC)


def _enc24(metres: float) -> bytes:
    raw = int(round(metres * POS_SCALE)) & 0xFFFFFF
    return bytes([raw & 0xFF, (raw >> 8) & 0xFF, (raw >> 16) & 0xFF])


def _dec24(b: bytes) -> float:
    v = b[0] | (b[1] << 8) | (b[2] << 16)
    if v & 0x800000:
        v -= 0x1000000  # sign-extend from 24-bit
    return v / POS_SCALE


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


class PoseFrame:
    @staticmethod
    def pack(pos_x: float, pos_y: float, pos_z: float,
             quat_w: float, quat_x: float, quat_y: float, quat_z: float) -> bytes:
        payload = (
            struct.pack("<H", MAGIC) +
            _enc24(pos_x) + _enc24(pos_y) + _enc24(pos_z) +
            struct.pack("<4h",
                        int(round(quat_w * QUAT_SCALE)),
                        int(round(quat_x * QUAT_SCALE)),
                        int(round(quat_y * QUAT_SCALE)),
                        int(round(quat_z * QUAT_SCALE)))
        )  # 19 bytes
        return payload + struct.pack("<H", crc16_ccitt(payload))  # 21 bytes

    @staticmethod
    def unpack(data: bytes) -> dict:
        if len(data) < FRAME_SIZE:
            raise ValueError(f"Too short: {len(data)} < {FRAME_SIZE}")
        payload  = data[:FRAME_SIZE - 2]
        recv_crc = struct.unpack_from("<H", data, FRAME_SIZE - 2)[0]
        if crc16_ccitt(payload) != recv_crc:
            raise ValueError("CRC mismatch")
        if struct.unpack_from("<H", payload, 0)[0] != MAGIC:
            raise ValueError("Bad magic")
        px = _dec24(payload[2:5])
        py = _dec24(payload[5:8])
        pz = _dec24(payload[8:11])
        qw, qx, qy, qz = struct.unpack_from("<4h", payload, 11)
        return dict(
            pos=(px, py, pz),
            quat=(qw / QUAT_SCALE, qx / QUAT_SCALE,
                  qy / QUAT_SCALE, qz / QUAT_SCALE)
        )
