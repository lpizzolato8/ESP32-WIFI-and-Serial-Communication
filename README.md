# ESP32-S3-N16R8 WiFi-UART Bridge

A complete ESP-IDF project that turns the ESP32-S3-N16R8 into a WiFi hotspot
with a **bidirectional** bridge between WiFi and UART — designed to bridge a
Linux laptop to an STM32 microcontroller wirelessly.

---

## How it works

```
                          ── Forward path (high-rate) ──

Linux laptop                        ESP32-S3                   Linux laptop
────────────                        ────────                   ────────────
test_bridge.py  ── WiFi UDP:4444 ──► bridge_task  ── UART TX GPIO17 ──► /dev/ttyUSB0


                          ── Reverse path (low-rate) ──

Linux laptop                        ESP32-S3                   Linux laptop
────────────                        ────────                   ────────────
/dev/ttyUSB0  ── UART RX GPIO18 ──► uart_udp_task ── WiFi UDP:4445 ──► test_bridge.py
```

**Forward path:** The Linux laptop sends binary pose frames (position +
quaternion) as UDP datagrams to the ESP32 soft-AP at `192.168.4.1:4444`. The
ESP32 writes the raw bytes verbatim to UART1 TX (GPIO17), which is wired to the
CP2102 USB-serial adapter and received on `/dev/ttyUSB0`.

**Reverse path:** The Linux laptop writes reverse frames to the serial port
(`/dev/ttyUSB0` → CP2102 → GPIO18). The ESP32 reads them from UART1 RX and
sends them as UDP datagrams to the laptop on port `4445`. The laptop's IP is
learned automatically from the first forward UDP packet received — no
hardcoding needed.

All four paths (UDP TX, serial RX, serial TX, UDP RX) run simultaneously on
separate threads in `test_bridge.py`.

---

## Hardware

| Spec        | Value          |
|-------------|----------------|
| Target      | ESP32-S3-N16R8 |
| Flash       | 16 MB Quad-SPI |
| PSRAM       | 8 MB Octal-SPI |
| IDF version | 6.1+           |

### USB ports

| Port | Linux device | Purpose |
|------|--------------|---------|
| USB-C (CH9102 CDC) | `/dev/ttyACM0` | IDF monitor / console (UART0) |
| USB-A (CP2102)     | `/dev/ttyUSB0` | Bidirectional frame data (UART1 GPIO17/18) |

### Wiring

| ESP32 Pin | Connects to   | Direction | Purpose                          |
|-----------|---------------|-----------|----------------------------------|
| GPIO17    | CP2102 RX     | ESP32 → laptop | UART1 TX — forward path     |
| GPIO18    | CP2102 TX     | laptop → ESP32 | UART1 RX — reverse path     |
| GND       | CP2102 GND    | —         | Common ground (required)         |

The rule is always **TX → RX, RX → TX** (crossed). GPIO17 is the ESP32's
transmit pin; it connects to the CP2102's receive pin, and vice versa.

**Future STM32 setup** — same pins, just connect to the STM32 instead of the
CP2102. No firmware change needed.

| ESP32 Pin | Connects to | Purpose          |
|-----------|-------------|------------------|
| GPIO17    | STM32 RX    | UART1 data out   |
| GPIO18    | STM32 TX    | UART1 data in    |
| GND       | STM32 GND   | Common ground    |

---

## Network settings

| Setting     | Value           |
|-------------|-----------------|
| SSID        | ESP32S3-Hotspot |
| Password    | esp32pass       |
| Auth mode   | WPA2-PSK        |
| ESP32 IP    | 192.168.4.1     |
| Forward UDP port | 4444       |
| Reverse UDP port | 4445       |
| Max clients | 4               |

To change SSID or password edit the define block at the top of `main/wifi_ap.c`.

---

## Frame formats

### Forward frame (laptop → ESP32 → serial) — 21 bytes

Defined in `pose_frame.py`. The ESP32 forwards all 21 bytes verbatim.

| Offset | Size | Field  | Type   | Encoding                              |
|--------|------|--------|--------|---------------------------------------|
| 0      | 2    | magic  | uint16 | 0xABCD little-endian — frame sync     |
| 2      | 3    | pos_x  | int24  | metres × 10000 (0.1 mm res, ±838 m)  |
| 5      | 3    | pos_y  | int24  | metres × 10000                        |
| 8      | 3    | pos_z  | int24  | metres × 10000                        |
| 11     | 2    | quat_w | int16  | component × 32767 (~0.00003 res)      |
| 13     | 2    | quat_x | int16  | component × 32767                     |
| 15     | 2    | quat_y | int16  | component × 32767                     |
| 17     | 2    | quat_z | int16  | component × 32767                     |
| 19     | 2    | crc16  | uint16 | CRC-16/CCITT-FALSE over bytes [0..18] |
| **21** |      |        |        |                                       |

### Reverse frame (laptop → ESP32 → UDP) — 138 bytes

Defined in `test_bridge.py` (`make_reverse_frame`).

| Offset | Size | Field   | Type   | Encoding                               |
|--------|------|---------|--------|----------------------------------------|
| 0      | 4    | magic   | bytes  | `REV\xAA` — frame sync                |
| 4      | 4    | counter | uint32 | little-endian, increments each frame   |
| 8      | 128  | payload | bytes  | counter repeated as uint32 LE (test pattern) |
| 136    | 2    | crc16   | uint16 | CRC-16/CCITT-FALSE over bytes [0..135] |
| **138**|      |         |        |                                        |

The receiver searches for the magic bytes and validates the CRC on every frame.
Bad frames increment `rev_crc_fail` rather than `rev_recv`.

---

## Project layout

```
ESP32Communication/
├── CMakeLists.txt           # Top-level ESP-IDF project file
├── sdkconfig.defaults       # N16R8 flash/PSRAM + WiFi config
├── partitions.csv           # Custom 16 MB partition table
├── pose_frame.py            # Python frame pack/unpack with CRC
├── test_bridge.py           # Bidirectional bridge test (4 threads)
├── read_serial.py           # Serial monitor with magic re-sync
├── test_send.py             # Minimal send-only test (no serial)
└── main/
    ├── CMakeLists.txt       # Component registration (IDF 6.1+)
    ├── main.c               # app_main boot sequence
    ├── wifi_ap.c            # Soft-AP initialisation
    ├── wifi_ap.h
    ├── udp_uart_bridge.c    # Bidirectional UDP↔UART bridge (2 tasks)
    ├── udp_uart_bridge.h
    └── pose_frame.h         # Compact binary frame struct for STM32
```

---

## Prerequisites

ESP-IDF v6.1+ — source it before using idf.py:

```bash
. ~/esp/esp-idf/export.sh
```

Add to `~/.bashrc` so it runs automatically in every terminal:

```bash
echo '. ~/esp/esp-idf/export.sh' >> ~/.bashrc
source ~/.bashrc
```

Python dependency:

```bash
pip install pyserial
```

---

## Build and flash

```bash
cd ~/ESP32Communication

# First time only
idf.py set-target esp32s3

# Build
idf.py build

# Flash and open monitor
idf.py -p /dev/ttyACM0 flash monitor
```

If you get CMake errors about a stale build:

```bash
rm -rf build/ sdkconfig
idf.py set-target esp32s3
idf.py build
```

Press `Ctrl+]` to exit the monitor.

---

## Finding the serial port

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Expected with both USB ports connected:
- `/dev/ttyACM0` — native USB CDC (flash/monitor)
- `/dev/ttyUSB0` — CP2102 USB-serial bridge (bidirectional frame data)

If you get permission denied:

```bash
sudo usermod -a -G dialout $USER
# log out and back in
```

---

## Expected monitor output on boot

```
I (604) wifi_ap: AP ready  IP: 192.168.4.1  Password: "esp32pass"
I (614) wifi_ap: Soft-AP started  SSID: "ESP32S3-Hotspot"  CH: 6
I (624) udp_uart: UART1 ready  baud=921600  TX=GPIO17  RX=GPIO18
I (634) udp_uart: UDP bridge listening on 192.168.4.1:4444
I (634) udp_uart: Reverse UDP task started, waiting for client address
I (634) main: Ready. Send UDP datagrams to 192.168.4.1:4444
```

Once the first forward packet arrives the reverse path opens:

```
I (xxx) udp_uart: Learned client address: 192.168.4.2 → reverse UDP to :4445
```

---

## Testing

### Step 1 — connect to the hotspot

```bash
nmcli dev wifi connect "ESP32S3-Hotspot" password "esp32pass" ifname wlan0
ping 192.168.4.1
```

### Step 2 — run the bridge test

`test_bridge.py` runs four threads simultaneously:

| Thread | Direction | Transport |
|--------|-----------|-----------|
| Forward send loop | Laptop → ESP32 | UDP port 4444 |
| `serial_receiver` | ESP32 → Laptop | UART → `/dev/ttyUSB0` |
| `serial_sender`   | Laptop → ESP32 | `/dev/ttyUSB0` → UART |
| `udp_receiver`    | ESP32 → Laptop | UDP port 4445 |

```bash
python3 test_bridge.py --port /dev/ttyUSB0 --baud 921600
```

Sample output:

```
[serial] opened /dev/ttyUSB0 @ 921600 baud
[udp-tx] sending to 192.168.4.1:4444
[udp-rx] listening on :4445
[run]    250 frames @ 250 Hz  (frame=21B)
[run]    reverse channel @ 10 Hz

  frame     1  latency=5.81ms  pos=(1.0000, 2.0000, 3.0000)
  ...
[rev-udp] frame      0  138B  CRC OK
[rev-udp] frame      1  138B  CRC OK
  ...
── Results ────────────────────────────────────────────────────────────────
  frames sent     : 250
  frames received : 250  (CRC OK)
  CRC failures    : 0
  never arrived   : 0  (lost in WiFi/UART)
  packet loss     : 0  (0.0%)
  latency avg     : 4.66ms
  latency p95     : 10.17ms

── Reverse channel ────────────────────────────────
  rev frames sent : 2
  rev frames recv : 2  (CRC OK)
  rev CRC failures: 0
  rev packet loss : 0  (0.0%)
```

### Step 3 — monitor only (no send)

```bash
python3 read_serial.py
```

---

## Tunable parameters

### main/udp_uart_bridge.c

| Define           | Value      | Description                          |
|------------------|------------|--------------------------------------|
| BRIDGE_UDP_PORT  | 4444       | ESP32 listens for forward frames     |
| REVERSE_UDP_PORT | 4445       | Laptop listens for reverse frames    |
| UART_BAUD_RATE   | 921600     | Must match both ends                 |
| UART_TX_PIN      | 17         | GPIO17 TX → CP2102 RX                |
| UART_RX_PIN      | 18         | GPIO18 RX ← CP2102 TX                |

### test_bridge.py

| Argument    | Default      | Description                        |
|-------------|--------------|------------------------------------|
| --port      | /dev/ttyUSB0 | Serial port                        |
| --baud      | 921600       | Must match firmware                |
| --hz        | 250          | Forward send rate                  |
| --count     | 250          | Number of forward frames to send   |
| --rev-hz    | 10           | Reverse send rate                  |
| --rev-port  | 4445         | UDP port to receive reverse frames |

### main/wifi_ap.c

| Define         | Default         | Description              |
|----------------|-----------------|--------------------------|
| AP_SSID        | ESP32S3-Hotspot | Network name             |
| AP_PASSWORD    | esp32pass       | Password (min 8 chars)   |
| AP_CHANNEL     | 6               | WiFi channel 1–13        |
| AP_MAX_CLIENTS | 4               | Max simultaneous clients |

---

## Latency targets for drone control

| Control loop           | Acceptable | Achieved (forward path)      |
|------------------------|------------|------------------------------|
| Position / velocity    | < 50 ms    | avg ~4.7 ms                  |
| Pilot command input    | < 20 ms    | avg ~4.7 ms, p95 ~10 ms      |
| Attitude stabilisation | < 10 ms    | baseline ~3 ms, spikes ~20 ms|
