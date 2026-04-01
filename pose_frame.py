import struct

POSE_FRAME_MAGIC = 0xABCD
FRAME_FORMAT     = "<HH7f"
_PAYLOAD_SIZE    = struct.calcsize(FRAME_FORMAT)  # 32 bytes (no CRC)
FRAME_SIZE       = _PAYLOAD_SIZE + 2              # 34 bytes (+ 2-byte CRC16)


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
    def pack(seq, pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z) -> bytes:
        payload = struct.pack(FRAME_FORMAT,
                              POSE_FRAME_MAGIC, seq,
                              pos_x, pos_y, pos_z,
                              quat_w, quat_x, quat_y, quat_z)
        crc = crc16_ccitt(payload)
        return payload + struct.pack("<H", crc)

    @staticmethod
    def unpack(data: bytes) -> dict:
        """Unpack and CRC-validate a frame. Raises ValueError on bad magic or CRC."""
        if len(data) < FRAME_SIZE:
            raise ValueError(f"Too short: {len(data)} < {FRAME_SIZE}")
        payload = data[:_PAYLOAD_SIZE]
        (recv_crc,) = struct.unpack_from("<H", data, _PAYLOAD_SIZE)
        calc_crc = crc16_ccitt(payload)
        if calc_crc != recv_crc:
            raise ValueError(f"CRC mismatch: got 0x{recv_crc:04X}, expected 0x{calc_crc:04X}")
        magic, seq, px, py, pz, qw, qx, qy, qz = struct.unpack_from(FRAME_FORMAT, payload)
        if magic != POSE_FRAME_MAGIC:
            raise ValueError(f"Bad magic: 0x{magic:04X}")
        return dict(seq=seq, pos=(px, py, pz), quat=(qw, qx, qy, qz))
