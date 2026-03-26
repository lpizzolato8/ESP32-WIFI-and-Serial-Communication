import serial
import struct

PORT     = "COM7"       # change to your port
BAUD     = 115200
MAGIC    = 0xABCD
FMT      = "<HH7f"
SIZE     = struct.calcsize(FMT)  # 32 bytes

s = serial.Serial(PORT, BAUD, timeout=1)
print(f"Listening on {PORT}...")

buf = b""
while True:
    buf += s.read(SIZE)
    if len(buf) >= SIZE:
        magic, seq, px, py, pz, qw, qx, qy, qz = struct.unpack_from(FMT, buf)
        if magic == MAGIC:
            print(f"seq={seq:>5}  pos=({px:.3f}, {py:.3f}, {pz:.3f})  "
                  f"quat=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})")
        buf = buf[SIZE:]