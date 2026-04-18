#!/usr/bin/env python3
"""
test_bridge.py  –  Bidirectional ESP32 bridge tester.

  Forward  (high-rate): Laptop ──UDP──► ESP32 ──UART──► Laptop serial RX
  Reverse  (low-rate) : Laptop ──UART──► ESP32 ──UDP──► Laptop UDP RX

All four paths run simultaneously in separate threads.

Usage:
    python3 test_bridge.py --port /dev/ttyUSB0 --baud 921600

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

ESP_IP            = "192.168.4.1"
ESP_UDP_PORT      = 4444          # ESP32 listens here (forward path)
REVERSE_UDP_PORT  = 4445          # Laptop listens here (reverse path)

SEND_COUNT        = 250
HZ                = 250
REVERSE_HZ        = 10            # Reverse channel send rate

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame() -> bytes:
    return PoseFrame.pack(pos_x=1.0, pos_y=2.0, pos_z=3.0,
                          quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)


# Reverse frame: 4-byte magic + 4-byte counter + 128-byte payload + 2-byte CRC-16
# Total: 138 bytes  (1104 bits)
_REV_MAGIC        = b"REV\xAA"
REV_PAYLOAD_SIZE  = 128
REV_FRAME_SIZE    = len(_REV_MAGIC) + 4 + REV_PAYLOAD_SIZE + 2  # 138

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc

def make_reverse_frame(counter: int) -> bytes:
    # 128-byte payload: counter repeated as uint32 LE to fill the space
    payload = (struct.pack("<I", counter & 0xFFFFFFFF) * (REV_PAYLOAD_SIZE // 4))
    body = _REV_MAGIC + struct.pack("<I", counter & 0xFFFFFFFF) + payload
    return body + struct.pack("<H", _crc16(body))


# ── Forward serial receiver (ESP32 → UART → Laptop) ─────────────────────────

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


# ── Reverse serial sender (Laptop → UART → ESP32) ───────────────────────────

def serial_sender(ser, stop_event: threading.Event, hz: float, counters: dict):
    delay = 1.0 / hz
    counter = 0
    while not stop_event.is_set():
        t0 = time.perf_counter()
        frame = make_reverse_frame(counter)
        ser.write(frame)
        counters["rev_sent"] += 1
        counter += 1
        remaining = delay - (time.perf_counter() - t0)
        if remaining > 0.0005:
            time.sleep(remaining - 0.0005)
        while time.perf_counter() < t0 + delay:
            pass


# ── Reverse UDP receiver (ESP32 → WiFi/UDP → Laptop) ────────────────────────

def udp_receiver(sock: socket.socket, stop_event: threading.Event, counters: dict):
    sock.settimeout(0.1)
    while not stop_event.is_set():
        try:
            # Each recvfrom() returns exactly one datagram = one complete frame.
            data, _ = sock.recvfrom(512)
            if len(data) < REV_FRAME_SIZE or data[:4] != _REV_MAGIC:
                continue
            body, recv_crc = data[:REV_FRAME_SIZE - 2], struct.unpack("<H", data[REV_FRAME_SIZE - 2:REV_FRAME_SIZE])[0]
            if _crc16(body) == recv_crc:
                counter_val = struct.unpack("<I", data[4:8])[0]
                counters["rev_recv"] += 1
                print(f"[rev-udp] frame {counter_val:>6}  {REV_FRAME_SIZE}B  CRC OK")
            else:
                counters["rev_crc_fail"] += 1
        except socket.timeout:
            pass


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ESP32 bidirectional UDP↔UART bridge tester")
    parser.add_argument("--port",       default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud",       default=921600, type=int,   help="Baud rate")
    parser.add_argument("--count",      default=SEND_COUNT, type=int, help="Forward frames to send")
    parser.add_argument("--hz",         default=HZ, type=float,    help="Forward send rate in Hz")
    parser.add_argument("--rev-hz",     default=REVERSE_HZ, type=float, help="Reverse send rate in Hz")
    parser.add_argument("--rev-port",   default=REVERSE_UDP_PORT, type=int,
                        help="ESP32 TCP port for reverse data (default 4445)")
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

    # Forward UDP socket (laptop → ESP32)
    udp_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[udp-tx] sending to {ESP_IP}:{ESP_UDP_PORT}")

    # Reverse UDP socket (ESP32 → laptop)
    udp_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_rx.bind(("", args.rev_port))
    print(f"[udp-rx] listening on :{args.rev_port}")

    print(f"[run]    {args.count} frames @ {args.hz:.0f} Hz  (frame={FRAME_SIZE}B)")
    print(f"[run]    reverse channel @ {args.rev_hz:.0f} Hz\n")

    send_times  = []
    recv_times  = []
    stop_event  = threading.Event()
    counters    = {"raw_bytes": 0, "crc_fail": 0, "rev_sent": 0, "rev_recv": 0, "rev_crc_fail": 0}

    # Thread 1: serial RX  (ESP32 → UART → Laptop)
    rx_thread = threading.Thread(target=serial_receiver,
                                 args=(ser, recv_times, send_times, stop_event, counters),
                                 daemon=True)
    # Thread 2: serial TX  (Laptop → UART → ESP32)
    tx_rev_thread = threading.Thread(target=serial_sender,
                                     args=(ser, stop_event, args.rev_hz, counters),
                                     daemon=True)
    # Thread 3: UDP RX     (ESP32 → WiFi/UDP → Laptop)
    rx_rev_thread = threading.Thread(target=udp_receiver,
                                     args=(udp_rx, stop_event, counters),
                                     daemon=True)

    rx_thread.start()
    tx_rev_thread.start()
    rx_rev_thread.start()

    # ── Forward send loop (Laptop → WiFi → ESP32) ───────────────────────────
    for _ in range(args.count):
        frame = make_frame()
        send_times.append(time.perf_counter())
        udp_tx.sendto(frame, (ESP_IP, ESP_UDP_PORT))
        next_send = send_times[-1] + send_delay
        remaining = next_send - time.perf_counter()
        if remaining > 0.0005:
            time.sleep(remaining - 0.0005)
        while time.perf_counter() < next_send:
            pass

    time.sleep(0.5)
    stop_event.set()
    rx_thread.join(timeout=1.0)
    tx_rev_thread.join(timeout=1.0)
    rx_rev_thread.join(timeout=1.0)

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

    print(f"\n── Reverse channel {'─'*28}")
    rev_s = counters['rev_sent']
    rev_r = counters['rev_recv']
    print(f"  rev frames sent : {rev_s}")
    print(f"  rev frames recv : {rev_r}  (CRC OK)")
    print(f"  rev CRC failures: {counters['rev_crc_fail']}")
    print(f"  rev packet loss : {rev_s - rev_r}  ({100*(rev_s-rev_r)/max(rev_s,1):.1f}%)")

    udp_tx.close()
    udp_rx.close()
    ser.close()


if __name__ == "__main__":
    main()
