#!/usr/bin/env python3
"""
test_bridge.py  –  Send binary frames over UDP to the ESP32 bridge,
                   read them back from the local serial port, and
                   report packet loss and latency.

Usage:
    python3 test_bridge.py --port /dev/ttyUSB0 --baud 921600 --hz 300

Dependencies:
    pip install pyserial
"""
from pose_frame import PoseFrame, FRAME_SIZE
import argparse
import socket
import serial
import time
import struct
import threading
import sys

# ── Configuration ────────────────────────────────────────────────────────────

ESP_IP       = "192.168.4.1"
ESP_UDP_PORT = 4444

SEND_COUNT   = 300     # frames to send per run
HZ           = 300     # target send rate

MAGIC        = 0xABCD
MAGIC_BYTES  = struct.pack("<H", MAGIC)

# ESP32 forwards the payload only (no CRC appended)
SERIAL_FMT   = "<HH7f"
SERIAL_SIZE  = struct.calcsize(SERIAL_FMT)  # 32 bytes

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame(seq: int) -> bytes:
    return PoseFrame.pack(seq=seq,
                          pos_x=1.0, pos_y=2.0, pos_z=3.0,
                          quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)


# ── Receiver thread ──────────────────────────────────────────────────────────

def serial_receiver(ser, received_seqs: dict, send_times: dict, stop_event: threading.Event):
    """Reads frames from serial, records arrival time keyed by seq number."""
    buf = b""
    raw_bytes = 0
    while not stop_event.is_set():
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            raw_bytes += len(chunk)
        while len(buf) >= SERIAL_SIZE:
            if buf[:2] != MAGIC_BYTES:
                buf = buf[1:]
                continue
            try:
                magic, seq, px, py, pz, qw, qx, qy, qz = struct.unpack_from(SERIAL_FMT, buf)
                t_recv = time.perf_counter()
                received_seqs[seq] = t_recv
                if seq in send_times:
                    latency_ms = (t_recv - send_times[seq]) * 1000
                    print(f"  seq={seq:>5}  latency={latency_ms:.2f}ms  "
                          f"pos=({px:.3f}, {py:.3f}, {pz:.3f})")
            except struct.error:
                pass
            buf = buf[SERIAL_SIZE:]
    print(f"[serial] raw bytes received: {raw_bytes}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ESP32 UDP→UART bridge tester")
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud",  default=115200, type=int, help="Baud rate")
    parser.add_argument("--count", default=SEND_COUNT, type=int, help="Frames to send")
    parser.add_argument("--hz",    default=HZ, type=float, help="Send rate in Hz")
    args = parser.parse_args()

    send_delay = 1.0 / args.hz

    max_hz = (args.baud / 10) / FRAME_SIZE
    if args.hz > max_hz * 0.8:
        print(f"[warn]   {args.hz:.0f} Hz exceeds 80% of UART capacity "
              f"({max_hz:.0f} Hz max at {args.baud} baud) — expect drops")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
        print(f"[serial] opened {args.port} @ {args.baud} baud")
    except serial.SerialException as e:
        print(f"[serial] ERROR: {e}")
        sys.exit(1)

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[udp]    sending to {ESP_IP}:{ESP_UDP_PORT}")
    print(f"[run]    {args.count} frames @ {args.hz:.0f} Hz  (frame={FRAME_SIZE}B)\n")

    received_seqs = {}
    stop_event = threading.Event()

    send_times = {}

    rx_thread = threading.Thread(target=serial_receiver,
                                 args=(ser, received_seqs, send_times, stop_event),
                                 daemon=True)
    rx_thread.start()

    # ── Send loop ────────────────────────────────────────────────────────────
    for seq in range(args.count):
        frame = make_frame(seq)
        send_times[seq] = time.perf_counter()
        udp.sendto(frame, (ESP_IP, ESP_UDP_PORT))
        next_send = send_times[seq] + send_delay
        # busy-wait for tight timing at high Hz
        while time.perf_counter() < next_send:
            pass

    # Wait for in-flight frames to arrive (up to 500ms after last send)
    time.sleep(0.5)
    stop_event.set()

    # ── Summary ──────────────────────────────────────────────────────────────
    latencies = []
    for seq, t_recv in received_seqs.items():
        if seq in send_times:
            latencies.append((t_recv - send_times[seq]) * 1000)

    received = len(received_seqs)
    lost = args.count - received

    print(f"── Results {'─'*36}")
    print(f"  frames sent     : {args.count}")
    print(f"  frames received : {received}")
    print(f"  packet loss     : {lost}  ({100*lost/args.count:.1f}%)")

    if latencies:
        avg = sum(latencies) / len(latencies)
        print(f"  latency avg     : {avg:.2f}ms")
        print(f"  latency min     : {min(latencies):.2f}ms")
        print(f"  latency max     : {max(latencies):.2f}ms")

    udp.close()
    ser.close()


if __name__ == "__main__":
    main()
