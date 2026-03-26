# ESP32-S3-N16R8 WiFi to UART Bridge

A complete ESP-IDF project that turns the ESP32-S3-N16R8 into a WiFi hotspot
that receives binary pose frames (position + quaternion) over UDP and forwards
them verbatim to a UART port — designed to bridge a Linux laptop to an STM32
microcontroller wirelessly.

---

## How it works

```
Linux laptop                  ESP32-S3                  STM32 / Windows
─────────────                 ────────────              ───────────────
pose_frame.py  ──UDP:4444──►  udp_uart_bridge  ──TX──►  UART RX
(test_bridge.py)              192.168.4.1               (read_serial.py)
```

1. The ESP32 hosts a WiFi access point (ESP32S3-Hotspot)
2. The Linux laptop connects to that network and sends UDP datagrams
3. Each datagram is written byte-for-byte to UART1 (GPIO17)
4. The STM32 or Windows laptop via CP2102 reads the frames over serial

---

## Hardware

| Spec        | Value          |
|-------------|----------------|
| Target      | ESP32-S3-N16R8 |
| Flash       | 16 MB Quad-SPI |
| PSRAM       | 8 MB Octal-SPI |
| IDF version | 6.1+           |

### Wiring

| ESP32 Pin | Connects to            | Purpose               |
|-----------|------------------------|-----------------------|
| GPIO17    | STM32 RX / CP2102 RX   | UART data out         |
| GPIO18    | STM32 TX / CP2102 TX   | UART data in (future) |
| GND       | STM32 GND / CP2102 GND | Common ground         |
| USB-C     | Linux laptop           | Power + monitor       |

Important: CP2102 must be set to 3.3V — the ESP32-S3 is not 5V tolerant.

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

To change SSID or password edit the define block at the top of main/wifi_ap.c.

---

## Frame format

Defined in main/pose_frame.h and mirrored in pose_frame.py.

| Offset | Size | Field  | Type    | Notes                           |
|--------|------|--------|---------|---------------------------------|
| 0      | 2    | magic  | uint16  | Always 0xABCD                   |
| 2      | 2    | seq    | uint16  | Rolling counter, wraps at 65535 |
| 4      | 4    | pos_x  | float32 | Metres, little-endian           |
| 8      | 4    | pos_y  | float32 |                                 |
| 12     | 4    | pos_z  | float32 |                                 |
| 16     | 4    | quat_w | float32 | Quaternion scalar               |
| 20     | 4    | quat_x | float32 |                                 |
| 24     | 4    | quat_y | float32 |                                 |
| 28     | 4    | quat_z | float32 |                                 |
| Total  | 32   |        |         |                                 |

The magic field lets the STM32 find frame boundaries in the UART stream
even after a reset or partial receive.

---

## Project layout

```
ESP32Communication/
├── CMakeLists.txt           # Top-level ESP-IDF project file
├── sdkconfig.defaults       # N16R8 flash/PSRAM + WiFi config
├── partitions.csv           # Custom 16 MB partition table
├── pose_frame.py            # Python mirror of pose_frame_t
├── test_bridge.py           # Full send + receive latency test
├── read_serial.py           # Windows serial reader
└── main/
    ├── CMakeLists.txt       # Component registration (IDF 6.1+)
    ├── main.c               # app_main boot sequence
    ├── wifi_ap.c            # Soft-AP initialisation
    ├── wifi_ap.h
    ├── udp_uart_bridge.c    # UDP to UART passthrough
    ├── udp_uart_bridge.h
    └── pose_frame.h         # Binary frame struct definition
```

---

## Prerequisites

ESP-IDF v6.1+ — source it before using idf.py:

```bash
. ~/esp/esp-idf/export.sh
```

Add to ~/.bashrc so it runs automatically in every terminal:

```bash
echo '. ~/esp/esp-idf/export.sh' >> ~/.bashrc
source ~/.bashrc
```

Python pyserial:

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

Press Ctrl + ] to exit the monitor.

---

## Expected monitor output on boot

```
I (604) wifi_ap: AP ready  IP: 192.168.4.1  Password: "esp32pass"
I (614) wifi_ap: Soft-AP started  SSID: "ESP32S3-Hotspot"  CH: 6
I (624) udp_uart: UART1 ready  baud=115200  TX=GPIO17  RX=GPIO18
I (634) udp_uart: bridge_task started
I (634) udp_uart: UDP bridge listening on 192.168.4.1:4444
I (634) main: Ready. Send UDP datagrams to 192.168.4.1:4444
```

When a client connects and packets arrive:

```
I (xxx) wifi_ap: Client connected   MAC: xx:xx:xx:xx:xx:xx  AID: 1
I (xxx) udp_uart: recvfrom returned 32
```

---

## Testing

### Step 1 — connect Linux laptop to hotspot

```bash
nmcli dev wifi connect "ESP32S3-Hotspot" password "esp32pass" ifname wlan0
ping 192.168.4.1
```

### Step 2 — send test frames from Linux

Open a second terminal tab (Ctrl+Shift+T) while the monitor runs in the first:

```bash
cd ~/ESP32Communication
python3 test_bridge.py --port /dev/ttyACM0 --baud 115200 --count 100
```

Or a quick 5-frame send-only test:

```bash
python3 - << 'PYEOF'
import socket, time
from pose_frame import PoseFrame
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(5):
    frame = PoseFrame.pack(i, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)
    sock.sendto(frame, ("192.168.4.1", 4444))
    print(f"sent frame {i}")
    time.sleep(0.1)
sock.close()
PYEOF
```

### Step 3 — read serial on Windows via CP2102

Install the CP2102 driver:
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

Find your COM port in PowerShell:

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
```

Edit read_serial.py and set PORT to your COM port (e.g. COM4), then run:

```bash
python read_serial.py
```

Expected output:

```
seq=    0  pos=(1.000, 2.000, 3.000)  quat=(1.000, 0.000, 0.000, 0.000)
seq=    1  pos=(1.000, 2.000, 3.000)  quat=(1.000, 0.000, 0.000, 0.000)
```

---

## Tunable parameters

### main/wifi_ap.c

| Define           | Default         | Description              |
|------------------|-----------------|--------------------------|
| AP_SSID          | ESP32S3-Hotspot | Network name             |
| AP_PASSWORD      | esp32pass       | Password (min 8 chars)   |
| AP_CHANNEL       | 6               | WiFi channel 1-13        |
| AP_MAX_CLIENTS   | 4               | Max simultaneous clients |

### main/udp_uart_bridge.c

| Define            | Default    | Description             |
|-------------------|------------|-------------------------|
| BRIDGE_UDP_PORT   | 4444       | UDP port to listen on   |
| UART_PORT_NUM     | UART_NUM_1 | Do not use UART0        |
| UART_BAUD_RATE    | 115200     | Match your STM32 config |
| UART_TX_PIN       | 17         | GPIO to STM32 RX        |
| UART_RX_PIN       | 18         | GPIO from STM32 TX      |

