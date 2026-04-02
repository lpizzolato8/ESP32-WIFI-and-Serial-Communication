import serial
from pose_frame import PoseFrame, FRAME_SIZE, MAGIC_BYTES

PORT = "/dev/ttyUSB0"    # CP2102 → frame data (IDF console is on /dev/ttyACM0)
BAUD = 115200

s = serial.Serial(PORT, BAUD, timeout=0.05)
print(f"Listening on {PORT} @ {BAUD} baud  (frame={FRAME_SIZE}B)...")

buf = b""
while True:
    chunk = s.read(max(1, s.in_waiting))
    if chunk:
        buf += chunk
    while len(buf) >= FRAME_SIZE:
        if buf[:2] != MAGIC_BYTES:
            buf = buf[1:]
            continue
        try:
            parsed = PoseFrame.unpack(buf[:FRAME_SIZE])
            p, q   = parsed["pos"], parsed["quat"]
            print(f"pos=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  "
                  f"quat=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})")
            buf = buf[FRAME_SIZE:]
        except ValueError:
            buf = buf[1:]  # CRC fail or bad magic — re-sync
