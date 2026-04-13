# ESP32-S3-N16R8 WiFi-UART Bridge

A complete ESP-IDF project that turns the ESP32-S3-N16R8 into a WiFi hotspot
that receives binary pose frames (position + quaternion) over UDP and forwards
them verbatim to a UART port — designed to bridge a Linux laptop to an STM32
microcontroller wirelessly.

---

## How it works

```
Linux laptop                        ESP32-S3                   Linux laptop
────────────                        ────────                   ────────────
test_bridge.py  ── WiFi UDP:4444 ──► udp_uart_bridge ── UART ──► /dev/ttyACM0
                   192.168.4.1



Linux laptop                  ESP32-S3                        Linux laptop
────────────                  ────────                        ────────────
/dev/ttyACM0  ── UART ──► udp_uart_bridge ── WiFi UDP:4444 ──► bridge_receive.py
                                               192.168.4.1
```

**Outbound path (send):** The Linux laptop sends binary pose frames as UDP
datagrams over WiFi to the ESP32 soft-AP at 192.168.4.1:4444.

**Inbound path (receive):** The ESP32 forwards the raw bytes verbatim over
UART1 (GPIO43/44) through the onboard CP2102 USB-serial chip back to the Linux
laptop, where they are read and CRC-validated on `/dev/ttyACM0`.

Both paths run simultaneously on separate threads in `test_bridge.py`.

---

## Hardware

| Spec        | Value          |
|-------------|----------------|
| Target      | ESP32-S3-N16R8 |
| Flash       | 16 MB Quad-SPI |
| PSRAM       | 8 MB Octal-SPI |
| IDF version | 6.1+           |

### USB ports

The Linux Laptop uses one USB-C port and one USB-A Port:

| Port label | Linux device  | Purpose                              |
|------------|---------------|--------------------------------------|
| USB (CH9102 CDC) | `/dev/ttyACM0` | IDF monitor / console (UART0 GPIO43/44) |
| UART (CP2102)    | `/dev/ttyUSB0` | Frame data (UART1 GPIO17/18)           |

### Wiring

**Current setup — GPIO17 wired to CP2102 RX (laptop testing):**

| ESP32 Pin | Connects to       | Purpose                            |
|-----------|-------------------|------------------------------------|
| GPIO17    | CP2102 RX pin     | UART1 TX → USB serial → /dev/ttyUSB0 |
| GPIO18    | (unused)          | UART1 RX — not connected           |
| GND       | CP2102 GND        | Common ground                      |

**Future STM32 setup** — same GPIO17/18 pins, just connect to the STM32 instead:

| ESP32 Pin | Connects to  | Purpose          |
|-----------|--------------|------------------|
| GPIO17    | STM32 RX     | UART1 data out   |
| GPIO18    | STM32 TX     | UART1 data in    |
| GND       | STM32 GND    | Common ground    |

No firmware change needed when switching to the STM32 — just rewire GPIO17 from
the CP2102 to the STM32 RX pin.
---

## Network settings

| Setting     | Value             |
|-------------|-------------------|
| SSID        | ESP32S3-Hotspot   |
| Password    | esp32pass         |
| Auth mode   | WPA2-PSK          |
| ESP32 IP    | 192.168.4.1       |
| UDP port    | 4444              |
| Max clients | 4                 |

To change SSID or password edit the define block at the top of `main/wifi_ap.c`.

---

## Frame format (Python → ESP32 → receiver)

Defined in `pose_frame.py`. The ESP32 forwards all 21 bytes verbatim.

| Offset | Size | Field  | Type   | Encoding                              |
|--------|------|--------|--------|---------------------------------------|
| 0      | 2    | magic  | uint16 | 0xABCD little-endian — frame sync     |
| 2      | 3    | pos_x  | int24  | metres × 10000 (0.1mm res, ±838m)    |
| 5      | 3    | pos_y  | int24  | metres × 10000                        |
| 8      | 3    | pos_z  | int24  | metres × 10000                        |
| 11     | 2    | quat_w | int16  | component × 32767 (~0.00003 res)      |
| 13     | 2    | quat_x | int16  | component × 32767                     |
| 15     | 2    | quat_y | int16  | component × 32767                     |
| 17     | 2    | quat_z | int16  | component × 32767                     |
| 19     | 2    | crc16  | uint16 | CRC-16/CCITT-FALSE over bytes [0..18] |
| **21** |      |        |        |                                       |

The magic field lets the receiver find frame boundaries in the UART stream
even after a reset or partial receive. Both `read_serial.py` and `test_bridge.py`
re-sync byte-by-byte on bad magic or CRC failure.

`main/pose_frame.h` defines the same fixed-point format as a C struct for the
STM32 to decode on the receiving end.

---

## Project layout

```
ESP32Communication/
├── CMakeLists.txt           # Top-level ESP-IDF project file
├── sdkconfig.defaults       # N16R8 flash/PSRAM + WiFi config
├── partitions.csv           # Custom 16 MB partition table
├── pose_frame.py            # Python frame pack/unpack with CRC
├── test_bridge.py           # Threaded send + receive latency test
├── read_serial.py           # Serial monitor with magic re-sync
├── test_send.py             # Minimal send-only test (no serial)
└── main/
    ├── CMakeLists.txt       # Component registration (IDF 6.1+)
    ├── main.c               # app_main boot sequence
    ├── wifi_ap.c            # Soft-AP initialisation
    ├── wifi_ap.h
    ├── udp_uart_bridge.c    # UDP → UART passthrough (no parsing)
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

# Flash and open monitor (native USB port)
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

Expected with both USB-C ports connected:
- `/dev/ttyACM0` — native USB CDC (flash/monitor/data)
- `/dev/ttyUSB0` — CP2102 USB-serial bridge

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
I (634) udp_uart: bridge_task started
I (634) udp_uart: UDP bridge listening on 192.168.4.1:4444
I (634) main: Ready. Send UDP datagrams to 192.168.4.1:4444
```

When a client connects and packets arrive:

```
I (xxx) wifi_ap: Client connected   MAC: xx:xx:xx:xx:xx:xx  AID: 1
I (xxx) udp_uart: recvfrom returned 34
```

---

## Testing

### Step 1 — connect Linux laptop to hotspot

```bash
nmcli dev wifi connect "ESP32S3-Hotspot" password "esp32pass" ifname wlan0
ping 192.168.4.1
```
If this returns an error similar to no name found, ignore

### Step 2 — run the bridge test

`test_bridge.py` sends frames over WiFi UDP and reads them back over serial on
a separate thread, measuring per-frame latency, CRC integrity, and packet loss.

```bash
cd ~/ESP32Communication
python3 test_bridge.py --port /dev/ttyUSB0 --baud 921600 --hz 250
```

Sample output (confirmed working — 250 Hz, 0% packet loss, 0 CRC failures):

```
[serial] opened /dev/ttyUSB0 @ 921600 baud
[udp]    sending to 192.168.4.1:4444
[run]    250 frames @ 250 Hz  (frame=21B)

  frame     1  latency=5.81ms  pos=(1.0000, 2.0000, 3.0000)
  frame     2  latency=4.19ms  pos=(1.0000, 2.0000, 3.0000)
  ...
  frame   250  latency=6.55ms  pos=(1.0000, 2.0000, 3.0000)
[serial] raw bytes received: 5250
── Results ────────────────────────────────────────────────────────────────
  frames sent     : 250
  frames received : 250  (CRC OK)
  CRC failures    : 0
  never arrived   : 0  (lost in WiFi/UART)
  packet loss     : 0  (0.0%)
  latency avg     : 4.66ms
  latency min     : 2.83ms
  latency p95     : 10.17ms
  latency max     : 20.44ms
```

The occasional latency spike is normal WiFi jitter. Baseline latency is ~3ms.
p95 latency is under 11ms — well within drone control requirements.

A `[warn]` line is printed if the requested Hz exceeds 80% of UART capacity.

### Step 3 — monitor only (no send)

`read_serial.py` prints frames as they arrive without sending anything:

```bash
python3 read_serial.py   # PORT defaults to /dev/ttyUSB0
```

Expected output:

```
pos=(1.0000, 2.0000, 3.0000)  quat=(1.0000, 0.0000, 0.0000, 0.0000)
pos=(1.0000, 2.0000, 3.0000)  quat=(1.0000, 0.0000, 0.0000, 0.0000)
```

---

## Tunable parameters

### main/wifi_ap.c

| Define         | Default         | Description              |
|----------------|-----------------|--------------------------|
| AP_SSID        | ESP32S3-Hotspot | Network name             |
| AP_PASSWORD    | esp32pass       | Password (min 8 chars)   |
| AP_CHANNEL     | 6               | WiFi channel 1-13        |
| AP_MAX_CLIENTS | 4               | Max simultaneous clients |

### main/udp_uart_bridge.c

| Define          | Value      | Description                              |
|-----------------|------------|------------------------------------------|
| BRIDGE_UDP_PORT | 4444       | UDP port to listen on                    |
| UART_PORT_NUM   | UART_NUM_1 | UART instance                            |
| UART_BAUD_RATE  | 115200     | Must match receiver baud rate            |
| UART_TX_PIN     | 17         | GPIO17 TX → CP2102 RX (or STM32 RX)     |
| UART_RX_PIN     | 18         | GPIO18 RX ← unused (or STM32 TX)        |

### test_bridge.py

| Argument | Default       | Description                               |
|----------|---------------|-------------------------------------------|
| --port   | /dev/ttyUSB0  | Serial port (CP2102, frame data)          |
| --baud   | 921600        | Must match firmware                       |
| --hz     | 250           | Send rate (90% capacity = 3942 Hz at 921600 baud with 21B frames) |
| --count  | 250           | Number of frames to send                  |

### Latency targets for drone control

| Control loop          | Acceptable latency | Achieved                        |
|-----------------------|--------------------|----------------------------------|
| Position / velocity   | < 50ms             | ✓ avg 4.7ms                     |
| Pilot command input   | < 20ms             | ✓ avg 4.7ms, p95 10ms           |
| Attitude stabilisation| < 10ms             | ✓ baseline ~3ms, spikes to ~20ms |
