import serial
import struct

PORT     = "COM7"       # change to your port
BAUD     = 921600
MAGIC    = 0xABCD
FMT      = "<HH7f"
SIZE     = struct.calcsize(FMT)  # 32 bytes
MAGIC_BYTES = struct.pack("<H", MAGIC)

s = serial.Serial(PORT, BAUD, timeout=0.05)
print(f"Listening on {PORT}...")

buf = b""
while True:
    chunk = s.read(max(1, s.in_waiting))
    if chunk:
        buf += chunk
    while len(buf) >= SIZE:
        if buf[:2] != MAGIC_BYTES:
            # re-sync: advance one byte at a time until magic aligns
            buf = buf[1:]
            continue
        magic, seq, px, py, pz, qw, qx, qy, qz = struct.unpack_from(FMT, buf)
        print(f"seq={seq:>5}  pos=({px:.3f}, {py:.3f}, {pz:.3f})  "
              f"quat=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})")
        buf = buf[SIZE:]