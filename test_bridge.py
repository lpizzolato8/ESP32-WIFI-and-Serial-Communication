#!/usr/bin/env python3
"""
test_bridge.py  –  Send binary frames over UDP to the ESP32 bridge,
                   read them back from the local serial port, and
                   measure round-trip latency.

Usage:
    python3 test_bridge.py --port /dev/ttyUSB0 --baud 115200

Dependencies:
    pip install pyserial
"""

import argparse
import socket
import struct
import serial
import time
import sys

# ── Configuration ────────────────────────────────────────────────────────────

ESP_IP      = "192.168.4.1"
ESP_UDP_PORT = 4444

# Dummy frame: 3× position floats + 4× quaternion floats = 28 bytes
# Replace this struct with your actual frame definition.
FRAME_FORMAT = "<7f"   # little-endian, 7 floats
FRAME_SIZE   = struct.calcsize(FRAME_FORMAT)   # 28 bytes

SEND_COUNT   = 100     # frames to send per run
SEND_DELAY_S = 0.01    # 10 ms between frames → ~100 Hz

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame(seq: int) -> bytes:
    """Build a dummy pose frame. Swap in your real data here."""
    px, py, pz   = seq * 0.001, seq * 0.002, seq * 0.003   # position
    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0                   # identity quaternion
    return struct.pack(FRAME_FORMAT, px, py, pz, qw, qx, qy, qz)


def parse_frame(data: bytes):
    if len(data) < FRAME_SIZE:
        return None
    return struct.unpack(FRAME_FORMAT, data[:FRAME_SIZE])


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ESP32 UDP→UART bridge tester")
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud",  default=115200, type=int, help="Baud rate")
    parser.add_argument("--count", default=SEND_COUNT, type=int, help="Frames to send")
    args = parser.parse_args()

    # Open serial port (your laptop end, reading what the ESP32 forwarded)
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"[serial] opened {args.port} @ {args.baud} baud")
    except serial.SerialException as e:
        print(f"[serial] ERROR: {e}")
        sys.exit(1)

    # Open UDP socket
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.settimeout(0.5)
    print(f"[udp]    sending to {ESP_IP}:{ESP_UDP_PORT}")
    print(f"[frame]  format={FRAME_FORMAT}  size={FRAME_SIZE} bytes")
    print(f"[run]    sending {args.count} frames at ~{1/SEND_DELAY_S:.0f} Hz\n")

    latencies = []

    for seq in range(args.count):
        frame = make_frame(seq)

        t_send = time.perf_counter()
        udp.sendto(frame, (ESP_IP, ESP_UDP_PORT))

        # Read back from serial
        received = b""
        deadline = time.perf_counter() + 0.5  # 500 ms timeout per frame
        while len(received) < FRAME_SIZE and time.perf_counter() < deadline:
            chunk = ser.read(FRAME_SIZE - len(received))
            received += chunk

        t_recv = time.perf_counter()

        if len(received) >= FRAME_SIZE:
            latency_ms = (t_recv - t_send) * 1000
            latencies.append(latency_ms)
            parsed = parse_frame(received)
            print(f"  frame {seq:>4}  {latency_ms:6.2f} ms  pos=({parsed[0]:.4f}, {parsed[1]:.4f}, {parsed[2]:.4f})")
        else:
            print(f"  frame {seq:>4}  TIMEOUT (got {len(received)}/{FRAME_SIZE} bytes)")

        time.sleep(SEND_DELAY_S)

    # ── Summary ──────────────────────────────────────────────────────────
    if latencies:
        avg = sum(latencies) / len(latencies)
        mn  = min(latencies)
        mx  = max(latencies)
        print(f"\n── Results ──────────────────────────────────")
        print(f"  frames sent   : {args.count}")
        print(f"  frames received: {len(latencies)}")
        print(f"  loss          : {args.count - len(latencies)}")
        print(f"  latency avg   : {avg:.2f} ms")
        print(f"  latency min   : {mn:.2f} ms")
        print(f"  latency max   : {mx:.2f} ms")

    udp.close()
    ser.close()


if __name__ == "__main__":
    main()