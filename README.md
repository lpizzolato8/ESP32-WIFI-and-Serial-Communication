# ESP32-S3-N16R8 WiFi to UART Bridge

A complete ESP-IDF project that turns the ESP32-S3-N16R8 into a WiFi hotspot
that receives binary pose frames (position + quaternion) over UDP and forwards
them verbatim to a UART port — designed to bridge a Linux laptop to an STM32
microcontroller wirelessly.

---

## How it works

```
Linux laptop                  ESP32-S3                  Receiver
─────────────                 ────────────              ────────
pose_frame.py  ──UDP:4444──►  udp_uart_bridge  ──TX──►  UART RX
test_bridge.py                192.168.4.1
```

1. The ESP32 hosts a WiFi access point (ESP32S3-Hotspot)
2. The Linux laptop connects to that network and sends UDP datagrams
3. Each datagram is written byte-for-byte to UART — no parsing, no validation
4. The receiver reads frames over serial

The ESP32 also sends a 1-byte UDP ack (`0xAC`) back to the sender before writing
to UART, allowing the sender to measure WiFi round-trip latency separately from
UART latency.

---

## Hardware

| Spec        | Value          |
|-------------|----------------|
| Target      | ESP32-S3-N16R8 |
| Flash       | 16 MB Quad-SPI |
| PSRAM       | 8 MB Octal-SPI |
| IDF version | 6.1+           |

### UART modes

There are two UART configurations depending on what is receiving the data.

**Laptop testing mode (current)** — data readable on `/dev/ttyUSB0`:

| ESP32 Pin | Connects to      | Purpose                        |
|-----------|------------------|--------------------------------|
| GPIO43    | USB-serial chip  | UART0 TX → laptop via USB      |
| GPIO44    | USB-serial chip  | UART0 RX ← laptop via USB      |
| USB-C     | Linux laptop     | Power + serial + monitor       |

**STM32 mode (future)** — swap these defines in `udp_uart_bridge.c`:

| ESP32 Pin | Connects to  | Purpose               |
|-----------|--------------|-----------------------|
| GPIO17    | STM32 RX     | UART1 data out        |
| GPIO18    | STM32 TX     | UART1 data in (future)|
| GND       | STM32 GND    | Common ground         |
| USB-C     | Linux laptop | Power + monitor       |

To switch to STM32 mode, change these defines in `main/udp_uart_bridge.c`:

```c
#define UART_PORT_NUM    UART_NUM_1
#define UART_TX_PIN      17
#define UART_RX_PIN      18
```

Note: CP2102 or STM32 GPIO must be 3.3V — the ESP32-S3 is not 5V tolerant.

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

Defined in `pose_frame.py`. The ESP32 forwards all 34 bytes verbatim.

| Offset | Size | Field  | Type    | Notes                           |
|--------|------|--------|---------|---------------------------------|
| 0      | 2    | magic  | uint16  | Always 0xABCD, little-endian    |
| 2      | 2    | seq    | uint16  | Rolling counter, wraps at 65535 |
| 4      | 4    | pos_x  | float32 | Metres, little-endian           |
| 8      | 4    | pos_y  | float32 |                                 |
| 12     | 4    | pos_z  | float32 |                                 |
| 16     | 4    | quat_w | float32 | Quaternion scalar               |
| 20     | 4    | quat_x | float32 |                                 |
| 24     | 4    | quat_y | float32 |                                 |
| 28     | 4    | quat_z | float32 |                                 |
| 32     | 2    | crc16  | uint16  | CRC-16/CCITT-FALSE over [0..31] |
| Total  | 34   |        |         |                                 |

The magic field lets the receiver find frame boundaries in the UART stream
even after a reset or partial receive. `read_serial.py` and `test_bridge.py`
both re-sync byte-by-byte when magic is not found.

Note: `main/pose_frame.h` defines a separate compact 20-byte format (int24
positions, int16 quaternions) intended for the STM32 to use when encoding its
own outbound frames. It is not used by the passthrough bridge.

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

The ESP32 typically appears as `/dev/ttyUSB0` or `/dev/ttyACM0`. If you get
permission denied:

```bash
sudo usermod -a -G dialout $USER
# log out and back in
```

---

## Expected monitor output on boot

```
I (604) wifi_ap: AP ready  IP: 192.168.4.1  Password: "esp32pass"
I (614) wifi_ap: Soft-AP started  SSID: "ESP32S3-Hotspot"  CH: 6
I (624) udp_uart: UART0 ready  baud=921600  TX=GPIO43  RX=GPIO44
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

### Step 2 — run the bridge test

`test_bridge.py` sends frames over UDP and reads them back from serial on a
separate thread, measuring per-frame latency and packet loss.

```bash
cd ~/ESP32Communication
python3 test_bridge.py --port /dev/ttyUSB0 --baud 921600 --hz 300 --count 300
```

The receiver thread prints each frame as it arrives:

```
  seq=    0  latency=3.21ms  pos=(1.000, 2.000, 3.000)
  seq=    1  latency=3.18ms  pos=(1.000, 2.000, 3.000)
  ...
── Results ────────────────────────────────────
  frames sent     : 300
  frames received : 300
  packet loss     : 0  (0.0%)
  latency avg     :  3.20ms
  latency min     :  2.90ms
  latency max     :  5.10ms
```

A `[warn]` line is printed if the requested Hz exceeds 80% of UART capacity.

### Step 3 — monitor only (no send)

`read_serial.py` prints frames as they arrive without sending anything:

```bash
python3 read_serial.py   # edit PORT at the top to match your port
```

Expected output:

```
seq=    0  pos=(1.000, 2.000, 3.000)  quat=(1.000, 0.000, 0.000, 0.000)
seq=    1  pos=(1.000, 2.000, 3.000)  quat=(1.000, 0.000, 0.000, 0.000)
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

| Define          | Current        | STM32 mode  | Description                    |
|-----------------|----------------|-------------|--------------------------------|
| BRIDGE_UDP_PORT | 4444           | 4444        | UDP port to listen on          |
| UART_PORT_NUM   | UART_NUM_0     | UART_NUM_1  | UART_NUM_0 = USB, 1 = GPIO     |
| UART_BAUD_RATE  | 921600         | 921600      | Must match receiver            |
| UART_TX_PIN     | 43             | 17          | TX GPIO                        |
| UART_RX_PIN     | 44             | 18          | RX GPIO                        |

### test_bridge.py

| Argument  | Default       | Description               |
|-----------|---------------|---------------------------|
| --port    | /dev/ttyUSB0  | Serial port               |
| --baud    | 921600        | Must match firmware        |
| --hz      | 300           | Send rate (max ~270Hz safe at 921600 baud) |
| --count   | 300           | Number of frames to send  |

### Latency targets for drone control

| Control loop          | Acceptable latency |
|-----------------------|--------------------|
| Position / velocity   | < 50ms             |
| Pilot command input   | < 20ms             |
| Attitude stabilisation| < 10ms             |
