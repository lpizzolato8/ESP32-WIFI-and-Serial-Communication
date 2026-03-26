import struct

POSE_FRAME_MAGIC = 0xABCD
FRAME_FORMAT     = "<HH7f"
FRAME_SIZE       = struct.calcsize(FRAME_FORMAT)  # 32 bytes


class PoseFrame:
    @staticmethod
    def pack(seq, pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z) -> bytes:
        return struct.pack(FRAME_FORMAT,
                           POSE_FRAME_MAGIC, seq,
                           pos_x, pos_y, pos_z,
                           quat_w, quat_x, quat_y, quat_z)

    @staticmethod
    def unpack(data: bytes):
        if len(data) < FRAME_SIZE:
            raise ValueError(f"Too short: {len(data)} < {FRAME_SIZE}")
        magic, seq, px, py, pz, qw, qx, qy, qz = struct.unpack_from(FRAME_FORMAT, data)
        if magic != POSE_FRAME_MAGIC:
            raise ValueError(f"Bad magic: 0x{magic:04X}")
        return dict(seq=seq, pos=(px,py,pz), quat=(qw,qx,qy,qz))
