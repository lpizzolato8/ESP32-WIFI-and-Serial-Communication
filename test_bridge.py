#!/usr/bin/env python3
"""
test_bridge.py  –  Bidirectional ESP32 bridge tester.

  Forward  (high-rate): Laptop ──UDP──► ESP32 ──UART──► Laptop serial RX
  Reverse  (low-rate) : Laptop ──UART──► ESP32 ──UDP──► Laptop UDP RX

All four paths run simultaneously in separate threads.

Usage:
    python3 test_bridge.py --port /dev/ttyUSB0 --baud 921600

    # STM32 on the ESP32's UART: the STM32 sends the reverse (status) frames.
    # Keep the CP2102 RX on ESP32 GPIO17 for forward latency, or --port none.
    python3 test_bridge.py --stm32 --port /dev/ttyUSB0

Dependencies:
    pip install pyserial
"""
from pose_frame import PoseFrame, FRAME_SIZE, MAGIC_BYTES, POS_SCALE
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
REVERSE_HZ        = 50            # Reverse channel send rate

# ── Helpers ──────────────────────────────────────────────────────────────────

def make_frame(seq: int) -> bytes:
    # pos_x carries the sequence number (one 0.1 mm step per frame) so each
    # received frame can be matched to its own send time, even after a loss.
    return PoseFrame.pack(pos_x=seq / POS_SCALE, pos_y=2.0, pos_z=3.0,
                          quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)


def frame_seq(parsed: dict) -> int:
    return round(parsed["pos"][0] * POS_SCALE)


# Reverse frame: 4-byte magic + 4-byte counter + 1024-byte payload + 2-byte CRC-16
# Total: 1034 bytes — 56% of UART capacity at 50 Hz / 921600 baud
_REV_MAGIC        = b"REV\xAA"
REV_PAYLOAD_SIZE  = 1024
REV_FRAME_SIZE    = len(_REV_MAGIC) + 4 + REV_PAYLOAD_SIZE + 2  # 1034

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc

# STM32 status at the start of the reverse payload (stm32/src/pose_rx.h)
STM_STATUS_FMT  = "<4s6IB3xI21s"
STM_STATUS_KEYS = ("tag uptime_ms frames_ok crc_fail bytes_skipped uart_errors "
                   "failsafe_count failsafe last_rx_age_ms last_pose").split()

def parse_stm_status(payload: bytes):
    if payload[:4] != b"STM1":
        return None
    return dict(zip(STM_STATUS_KEYS, struct.unpack_from(STM_STATUS_FMT, payload)))

def make_reverse_frame(counter: int) -> bytes:
    # 1024-byte payload: counter repeated as uint32 LE to fill the space
    payload = (struct.pack("<I", counter & 0xFFFFFFFF) * (REV_PAYLOAD_SIZE // 4))
    body = _REV_MAGIC + struct.pack("<I", counter & 0xFFFFFFFF) + payload
    return body + struct.pack("<H", _crc16(body))


# ── Forward serial receiver (ESP32 → UART → Laptop) ─────────────────────────

def serial_receiver(ser, received: dict, send_times: list,
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
                seq = frame_seq(parsed)
                if 0 <= seq < len(send_times) and seq not in received:
                    received[seq] = t_recv
                    latency_ms = (t_recv - send_times[seq]) * 1000
                    p = parsed["pos"]
                    print(f"  frame {seq:>5}  latency={latency_ms:.2f}ms  "
                          f"pos=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
                buf = buf[FRAME_SIZE:]
            except ValueError:
                counters["crc_fail"] += 1
                buf = buf[1:]
    print(f"[serial] raw bytes received: {counters['raw_bytes']}")


# ── Ack receiver (ESP32 → WiFi/UDP → Laptop) ─────────────────────────────────

def ack_receiver(sock: socket.socket, ack_times: list, stop_event: threading.Event):
    # The ESP32 replies 0xAC to each forward datagram before its UART write,
    # so send → ack is the WiFi round trip without the UART leg.
    sock.settimeout(0.1)
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(16)
            if data == b"\xAC":
                ack_times.append(time.perf_counter())
        except socket.timeout:
            pass


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

def udp_receiver(sock: socket.socket, stop_event: threading.Event, counters: dict,
                 stm: dict):
    sock.settimeout(0.1)
    while not stop_event.is_set():
        try:
            # Each recvfrom() returns exactly one datagram = one complete frame.
            data, _ = sock.recvfrom(2048)
            if len(data) < REV_FRAME_SIZE or data[:4] != _REV_MAGIC:
                continue
            body, recv_crc = data[:REV_FRAME_SIZE - 2], struct.unpack("<H", data[REV_FRAME_SIZE - 2:REV_FRAME_SIZE])[0]
            if _crc16(body) == recv_crc:
                counter_val = struct.unpack("<I", data[4:8])[0]
                counters["rev_recv"] += 1
                status = parse_stm_status(data[8:8 + REV_PAYLOAD_SIZE])
                if status is None:
                    print(f"[rev-udp] frame {counter_val:>6}  {REV_FRAME_SIZE}B  CRC OK")
                    continue
                stm.setdefault("first", (counter_val, status))
                stm["last"] = (counter_val, status)
                if counter_val % 25 == 0:
                    print(f"[stm32]   status {counter_val:>6}  ok={status['frames_ok']}  "
                          f"crc_fail={status['crc_fail']}  failsafe={status['failsafe']}")
            else:
                counters["rev_crc_fail"] += 1
        except socket.timeout:
            pass


# ── STM32 summary ─────────────────────────────────────────────────────────────

def print_stm32_summary(stm: dict, counters: dict, sent: int):
    base_ctr, base = stm["base"]
    last_ctr, last = stm["last"]
    delta = {k: last[k] - base[k] for k in
             ("frames_ok", "crc_fail", "bytes_skipped", "uart_errors", "failsafe_count")}
    seq = round(PoseFrame.unpack(last["last_pose"])["pos"][0] * POS_SCALE)
    first_ctr = stm["first"][0]
    expected = last_ctr - first_ctr + 1

    print(f"\n── STM32 receiver {'─'*29}")
    print(f"  poses validated : {delta['frames_ok']} / {sent}")
    print(f"  CRC failures    : {delta['crc_fail']}")
    print(f"  bytes skipped   : {delta['bytes_skipped']}  (re-sync)")
    print(f"  UART errors     : {delta['uart_errors']}")
    # The stream stopping at the end of the run trips failsafe once; that
    # one isn't a dropout.
    dropouts = delta["failsafe_count"] - last["failsafe"]
    print(f"  failsafe drops  : {dropouts}  (gaps > 100 ms during the run)")
    print(f"  last pose seq   : {seq}  (expected {sent - 1})")
    print(f"  status frames   : {counters['rev_recv']} received, "
          f"{expected - counters['rev_recv']} missing  (counter gaps)")
    print(f"  rev CRC failures: {counters['rev_crc_fail']}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ESP32 bidirectional UDP↔UART bridge tester")
    parser.add_argument("--port",       default="/dev/ttyUSB0",
                        help="Serial port, or 'none' to skip the forward serial check")
    parser.add_argument("--baud",       default=921600, type=int,   help="Baud rate")
    parser.add_argument("--count",      default=SEND_COUNT, type=int, help="Forward frames to send")
    parser.add_argument("--hz",         default=HZ, type=float,    help="Forward send rate in Hz")
    parser.add_argument("--rev-hz",     default=REVERSE_HZ, type=float, help="Reverse send rate in Hz")
    parser.add_argument("--rev-port",   default=REVERSE_UDP_PORT, type=int,
                        help="Laptop UDP port for reverse data (default 4445)")
    parser.add_argument("--stm32",      action="store_true",
                        help="STM32 is on the UART: read its status frames "
                             "instead of sending reverse frames")
    args = parser.parse_args()
    use_serial = args.port.lower() != "none"

    send_delay = 1.0 / args.hz

    max_hz = (args.baud / 10) / FRAME_SIZE
    if args.hz > max_hz * 0.8:
        print(f"[warn]   {args.hz:.0f} Hz exceeds 80% of UART capacity "
              f"({max_hz:.0f} Hz max at {args.baud} baud) — expect drops")

    if not use_serial and not args.stm32:
        print("[error]  --port none only makes sense with --stm32")
        sys.exit(1)

    ser = None
    if use_serial:
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
    if args.stm32:
        print("[run]    reverse channel: STM32 status frames\n")
    else:
        print(f"[run]    reverse channel @ {args.rev_hz:.0f} Hz\n")

    send_times  = []
    received    = {}   # seq → receive time
    ack_times   = []
    stop_event  = threading.Event()
    counters    = {"raw_bytes": 0, "crc_fail": 0, "rev_sent": 0, "rev_recv": 0, "rev_crc_fail": 0}
    stm         = {}   # "first" / "last": (counter, status) from STM32 frames

    # Thread 1: serial RX  (ESP32 → UART → Laptop)
    rx_thread = threading.Thread(target=serial_receiver,
                                 args=(ser, received, send_times, stop_event, counters),
                                 daemon=True)
    # Thread 2: serial TX  (Laptop → UART → ESP32)
    tx_rev_thread = threading.Thread(target=serial_sender,
                                     args=(ser, stop_event, args.rev_hz, counters),
                                     daemon=True)
    # Thread 3: UDP RX     (ESP32 → WiFi/UDP → Laptop)
    rx_rev_thread = threading.Thread(target=udp_receiver,
                                     args=(udp_rx, stop_event, counters, stm),
                                     daemon=True)

    # Thread 4: UDP ack RX (ESP32 → WiFi/UDP → Laptop)
    ack_thread = threading.Thread(target=ack_receiver,
                                  args=(udp_tx, ack_times, stop_event),
                                  daemon=True)

    if ser:
        rx_thread.start()
    if not args.stm32:
        tx_rev_thread.start()
    rx_rev_thread.start()
    ack_thread.start()

    if args.stm32:
        # Baseline the STM32's counters before sending, so earlier runs
        # don't count. The status goes out on the reverse path only once the
        # ESP32 knows the laptop's address, so send one frame to teach it.
        udp_tx.sendto(make_frame(0), (ESP_IP, ESP_UDP_PORT))
        deadline = time.perf_counter() + 2.0
        while "first" not in stm and time.perf_counter() < deadline:
            time.sleep(0.01)
        if "first" not in stm:
            print("[stm32]  ERROR: no status frames from the STM32 within 2 s "
                  "(check STM32 TX → ESP32 GPIO18 and that it's flashed)")
            stop_event.set()
            sys.exit(1)
        time.sleep(0.05)
        stm["base"] = stm["last"]
        ack_times.clear()
        print(f"[stm32]  baseline: ok={stm['base'][1]['frames_ok']}\n")

    # ── Forward send loop (Laptop → WiFi → ESP32) ───────────────────────────
    for seq in range(args.count):
        frame = make_frame(seq)
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
    if ser:
        rx_thread.join(timeout=1.0)
    if not args.stm32:
        tx_rev_thread.join(timeout=1.0)
    rx_rev_thread.join(timeout=1.0)
    ack_thread.join(timeout=1.0)

    # ── Summary ──────────────────────────────────────────────────────────────
    n_recv = len(received)
    lost   = args.count - n_recv

    latencies = [(t - send_times[seq]) * 1000 for seq, t in received.items()]

    print(f"── Results {'─'*36}")
    print(f"  frames sent     : {args.count}")
    if ser:
        print(f"  frames received : {n_recv}  (CRC OK)")
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

    # Acks carry no sequence number, so they are paired with sends by order.
    # That is only valid when every ack arrived.
    print(f"\n── WiFi round trip (ack) {'─'*22}")
    print(f"  acks received   : {len(ack_times)} / {args.count}")
    if ack_times and len(ack_times) == args.count:
        rtts = sorted((ack_times[i] - send_times[i]) * 1000 for i in range(args.count))
        rtt_avg = sum(rtts) / len(rtts)
        print(f"  wifi rtt avg    : {rtt_avg:.2f}ms")
        print(f"  wifi rtt p95    : {rtts[int(len(rtts) * 0.95)]:.2f}ms")
        if latencies:
            print(f"  est. split      : wifi ≈ {rtt_avg / 2:.2f}ms one-way, "
                  f"esp32+uart+usb ≈ {avg - rtt_avg / 2:.2f}ms")
    elif ack_times:
        print("  some acks lost; skipping rtt (acks are paired by order)")

    if args.stm32:
        print_stm32_summary(stm, counters, args.count)
        udp_tx.close()
        udp_rx.close()
        if ser:
            ser.close()
        return

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
