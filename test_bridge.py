#!/usr/bin/env python3
"""
test_bridge.py  –  Send binary frames over UDP to the ESP32 bridge,
                   read them back from the local serial port, and
                   report packet loss and latency.

Usage:
    python3 test_bridge.py --port /dev/ttyACM0 --baud 921600

Dependencies:
    pip install pyserial
"""
from pose_frame import PoseFrame, FRAME_SIZE, MAGIC_BYTES
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

SEND_COUNT   = 250
HZ           = 250

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame() -> bytes:
    return PoseFrame.pack(pos_x=1.0, pos_y=2.0, pos_z=3.0,
                          quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)


# ── Receiver thread ──────────────────────────────────────────────────────────

def serial_receiver(ser, recv_times: list, send_times: list,
                    stop_event: threading.Event, counters: dict):
    buf = b""
    dumped = False
    while not stop_event.is_set():
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            counters["raw_bytes"] += len(chunk)
            if not dumped and len(buf) >= 40:
                print(f"[debug]  first 40 bytes: {buf[:40].hex(' ')}")
                try:
                    print(f"[debug]  as ascii:      {buf[:40]!r}")
                except Exception:
                    pass
                dumped = True
        while len(buf) >= FRAME_SIZE:
            if buf[:2] != MAGIC_BYTES:
                buf = buf[1:]
                continue
            try:
                parsed = PoseFrame.unpack(buf[:FRAME_SIZE])
                t_recv = time.perf_counter()
                recv_times.append(t_recv)
                n = len(recv_times)
                if n <= len(send_times):
                    latency_ms = (t_recv - send_times[n - 1]) * 1000
                    p = parsed["pos"]
                    print(f"  frame {n:>5}  latency={latency_ms:.2f}ms  "
                          f"pos=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
                buf = buf[FRAME_SIZE:]
            except ValueError:
                counters["crc_fail"] += 1
                buf = buf[1:]
    print(f"[serial] raw bytes received: {counters['raw_bytes']}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ESP32 UDP→UART bridge tester")
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud",  default=921600, type=int,   help="Baud rate")
    parser.add_argument("--count", default=SEND_COUNT, type=int, help="Frames to send")
    parser.add_argument("--hz",    default=HZ, type=float,    help="Send rate in Hz")
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

    send_times  = []
    recv_times  = []
    stop_event  = threading.Event()
    counters    = {"raw_bytes": 0, "crc_fail": 0}

    rx_thread = threading.Thread(target=serial_receiver,
                                 args=(ser, recv_times, send_times, stop_event, counters),
                                 daemon=True)
    rx_thread.start()

    # ── Send loop ────────────────────────────────────────────────────────────
    for _ in range(args.count):
        frame = make_frame()
        send_times.append(time.perf_counter())
        udp.sendto(frame, (ESP_IP, ESP_UDP_PORT))
        next_send = send_times[-1] + send_delay
        remaining = next_send - time.perf_counter()
        if remaining > 0.0005:
            time.sleep(remaining - 0.0005)
        while time.perf_counter() < next_send:
            pass

    time.sleep(0.5)
    stop_event.set()
    rx_thread.join(timeout=1.0)

    # ── Summary ──────────────────────────────────────────────────────────────
    received = len(recv_times)
    lost     = args.count - received

    latencies = [(recv_times[i] - send_times[i]) * 1000
                 for i in range(min(received, args.count))]

    print(f"── Results {'─'*36}")
    print(f"  frames sent     : {args.count}")
    print(f"  frames received : {received}  (CRC OK)")
    print(f"  CRC failures    : {counters['crc_fail']}")
    print(f"  never arrived   : {lost - counters['crc_fail']}  (lost in WiFi/UART)")
    print(f"  packet loss     : {lost}  ({100*lost/args.count:.1f}%)")

    if latencies:
        avg = sum(latencies) / len(latencies)
        sorted_lat = sorted(latencies)
        p95 = sorted_lat[int(len(sorted_lat) * 0.95)]
        print(f"  latency avg     : {avg:.2f}ms")
        print(f"  latency min     : {min(latencies):.2f}ms")
        print(f"  latency p95     : {p95:.2f}ms")
        print(f"  latency max     : {max(latencies):.2f}ms")

    udp.close()
    ser.close()


if __name__ == "__main__":
    main()
