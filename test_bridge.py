#!/usr/bin/env python3
"""
test_bridge.py  –  Send binary frames over UDP to the ESP32 bridge,
                   read them back from the local serial port, and
                   measure latency in two legs:

   Leg 1 (WiFi RTT)  : laptop sendto() → ESP32 recvfrom() → ack back
                        measured as (t_ack - t_send)
   Leg 2 (UART leg)  : ESP32 writes UART → laptop reads serial
                        measured as (t_serial - t_ack)
   Total             : (t_serial - t_send)

   Note: the ESP32 sends the ack *before* uart_write_bytes(), so the ack
   always arrives before serial data — sequential waiting is safe.

Usage:
    python3 test_bridge.py --port /dev/ttyUSB0 --baud 115200

Dependencies:
    pip install pyserial
"""
from pose_frame import PoseFrame, FRAME_SIZE
import argparse
import socket
import serial
import time
import sys

# ── Configuration ────────────────────────────────────────────────────────────

ESP_IP       = "192.168.4.1"
ESP_UDP_PORT = 4444

SEND_COUNT   = 100     # frames to send per run
SEND_DELAY_S = 0.01    # 10 ms between frames → ~100 Hz

ACK_BYTE     = 0xAC    # must match udp_uart_bridge.c

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame(seq: int) -> bytes:
    return PoseFrame.pack(seq=seq,
                          pos_x=1.0, pos_y=2.0, pos_z=3.0,
                          quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)


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
    print(f"[udp]    sending to {ESP_IP}:{ESP_UDP_PORT}")
    print(f"[frame]  size={FRAME_SIZE} bytes")
    print(f"[run]    sending {args.count} frames at ~{1/SEND_DELAY_S:.0f} Hz\n")

    latencies   = []   # total
    wifi_rtts   = []
    uart_legs   = []

    for seq in range(args.count):
        frame = make_frame(seq)

        # ── Send ────────────────────────────────────────────────────────
        t_send = time.perf_counter()
        udp.sendto(frame, (ESP_IP, ESP_UDP_PORT))

        # ── Wait for UDP ack (Leg 1: WiFi round-trip) ────────────────────
        t_ack = None
        udp.settimeout(0.2)
        try:
            ack_data, _ = udp.recvfrom(16)
            if len(ack_data) >= 1 and ack_data[0] == ACK_BYTE:
                t_ack = time.perf_counter()
        except socket.timeout:
            pass

        # ── Wait for serial data (Leg 2: UART to laptop) ─────────────────
        received = b""
        deadline = time.perf_counter() + 0.5  # 500 ms timeout per frame
        while len(received) < FRAME_SIZE and time.perf_counter() < deadline:
            chunk = ser.read(FRAME_SIZE - len(received))
            received += chunk

        t_serial = time.perf_counter()

        # ── Report ───────────────────────────────────────────────────────
        byte_count = len(received)
        hex_dump   = received.hex(' ') if received else '(none)'

        print(f"\n  [frame {seq}]  bytes received: {byte_count}/{FRAME_SIZE}")
        print(f"  hex: {hex_dump}")

        if byte_count >= FRAME_SIZE:
            total_ms = (t_serial - t_send) * 1000
            latencies.append(total_ms)

            try:
                parsed   = PoseFrame.unpack(received)
                crc_ok   = True
                pos_str  = f"pos=({parsed['pos'][0]:.3f}, {parsed['pos'][1]:.3f}, {parsed['pos'][2]:.3f})"
                quat_str = f"quat=({parsed['quat'][0]:.3f}, {parsed['quat'][1]:.3f}, {parsed['quat'][2]:.3f}, {parsed['quat'][3]:.3f})"
            except ValueError as e:
                crc_ok   = False
                pos_str  = f"({e})"
                quat_str = ""

            crc_tag = "CRC OK" if crc_ok else "CRC FAIL"
            print(f"  {crc_tag}  {pos_str}  {quat_str}")

            if t_ack is not None:
                wifi_ms = (t_ack    - t_send)  * 1000
                uart_ms = (t_serial - t_ack)   * 1000
                wifi_rtts.append(wifi_ms)
                uart_legs.append(uart_ms)
                print(f"  total={total_ms:.2f}ms  wifi_rtt={wifi_ms:.2f}ms  uart_leg={uart_ms:.2f}ms")
            else:
                print(f"  total={total_ms:.2f}ms  (no ack received)")
        else:
            print(f"  TIMEOUT")

        time.sleep(SEND_DELAY_S)

    # ── Summary ──────────────────────────────────────────────────────────
    print(f"\n── Results {'─'*36}")
    print(f"  frames sent     : {args.count}")
    print(f"  frames received : {len(latencies)}")
    print(f"  packet loss     : {args.count - len(latencies)}")

    if latencies:
        def stats(label, data):
            if not data:
                return
            avg = sum(data) / len(data)
            print(f"  {label:<16}: avg={avg:6.2f}ms  min={min(data):6.2f}ms  max={max(data):6.2f}ms")

        stats("total latency",  latencies)
        stats("WiFi RTT",       wifi_rtts)
        stats("UART leg",       uart_legs)

    udp.close()
    ser.close()


if __name__ == "__main__":
    main()
