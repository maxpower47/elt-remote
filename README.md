# ELT Remote — Dual LoRa Beacon & Transmitter System

An end-to-end emergency location & remote beacon trigger system featuring **RAK4631** (nRF52840 + SX1262), **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262), and a **100% Offline Progressive Web App (PWA) Dashboard** via Web Bluetooth (NUS).

---

## 🛰 System Architecture

```
                                      +------------------------------------+
                                      |    Heltec WiFi LoRa 32 V3 (TX)     |
                                      |   (ESP32-S3 + SX1262 + SSD1306)   |
                                      +------------------------------------+
                                            /                        \
                            Web Bluetooth  /                          \  LoRa 915MHz
                             (NUS GATT)   /                            \  Telemetry & Commands
                                         v                              v
+--------------------------------------------------+          +------------------------------------+
| 📱 Mobile / Desktop Web PWA Dashboard            |          |     RAK4631 / RAK19003 (Beacon)    |
| (Offline Leaflet Maps, Controls, Battery Meters) |          | (3.3V MOSFET Siren + nRF52 BLE)    |
+--------------------------------------------------+          +------------------------------------+
```

---

## ⚡ Key Features

1. **Dual Device Architecture**:
   - **RAK4631 WisBlock Beacon**: Low-power nRF52840 MCU with non-volatile flash state persistence (NVM) and 3.3V logic gate driver for high-power MOSFET siren switch.
   - **Heltec V3 Handheld Transmitter**: OLED-driven handheld controller with 4-tab pixel art navigation icons, RSSI sparkline signal analyzer graph, and hold progress confirmation toasts.

2. **Handheld OLED Display Interface (Heltec V3)**:
   - **Tab 1 🏠 System Summary**: Live state, beacon battery gauge, link status, and real-time packet freshness (`Rx: Xs ago`).
   - **Tab 2 📍 GPS Telemetry**: Degrees Minutes Seconds (DMS) coordinates (`34°03'07.9"N`), Haversine distance, and compass heading (`Dist: 350m Hdg: 45° NE`).
   - **Tab 3 ⚙ Command Menu**: One-click menu selection for `ARM IMMEDIATELY`, `ARM WITH TIMER`, and `DISARM BEACON` with live button hold progress bar ($0\%-100\%$) and popup confirmation toast (`✓ COMMAND SENT!`).
   - **Tab 4 📶 Signal Analyzer**: Real-time LoRa signal metrics (RSSI, SNR, Link Health %) and a 30-sample live RSSI sparkline trend graph.

3. **100% Offline Progressive Web App (PWA)**:
   - Web Bluetooth API (Nordic UART Service - NUS) connects directly to either the Heltec Transmitter or RAK4631 Beacon.
   - Service Worker (`sw.js`) and PWA manifest (`manifest.json`) pre-cache all styling, assets, and Leaflet map tiles for complete offline field operation.

---

## 📌 Hardware Pinout & Wiring Mappings

### **RAK4631 / RAK19003 Mini Base Board**
| Pin Name | nRF52 GPIO | Function |
| :--- | :--- | :--- |
| **`TX` (J7 Header)** | `P0.20` | **3.3V MOSFET Gate Driver Output** |
| **`GND` (J7 Header)** | `GND` | **Common Logic Ground** |
| **`P1.03`** | `P1.03` | **Green LED** (RF Activity Pulse) |
| **`P1.04`** | `P1.04` | **Blue LED** (Armed / Active Status Pulse) |

#### **MOSFET Siren Switch Wiring**:
- Connect **`TX` Pad on RAK19003 J7 Header** $\rightarrow$ **MOSFET `TRIG` / `IN+`**.
- Connect **`GND` Pad on RAK19003 J7 Header** $\rightarrow$ **MOSFET `GND` / `IN-`**.
- Connect **External 12V Battery (+)** $\rightarrow$ **12V Siren Red (+) Wire**.
- Connect **External 12V Battery (-)** $\rightarrow$ **MOSFET `DC-` Terminal**.
- Connect **12V Siren Black (-) Wire** $\rightarrow$ **MOSFET `OUT-` Terminal**.

---

### **Heltec WiFi LoRa 32 V3**
| Pin Name | ESP32-S3 GPIO | Function |
| :--- | :--- | :--- |
| **`PRG` Button** | `GPIO0` | User Input Button (Short Click: Next / Long Press: Select) |
| **`VBAT ADC`** | `GPIO1` | Battery Voltage ADC (11dB Attenuation) |
| **`ADC CTRL`** | `GPIO37` | P-Channel MOSFET Divider Control (High-Impedance) |
| **`OLED SDA / SCL`** | `GPIO17 / GPIO18` | 0.96" SSD1306 OLED Display (I2C) |
| **`LoRa CS / RST / BUSY`** | `GPIO8 / GPIO12 / GPIO13` | SX1262 LoRa Radio Control Pins |

---

## 🛠 Compilation & Flashing Guide

Requires [PlatformIO CLI](https://platformio.org/):

### **1. Flash Heltec V3 Transmitter**
Connect Heltec V3 via USB cable:
```bash
pio run -e heltec_wifi_lora_32_V3 --target upload
```

### **2. Flash RAK4631 Beacon**
Double-tap RESET button on RAK4631 to enter DFU mode:
```bash
pio run -e wiscore_rak4631
adafruit-nrfutil dfu serial --package .pio/build/wiscore_rak4631/firmware.zip -p /dev/ttyACM0 -b 115200 --singlebank --touch 1200
```

---

## 🌐 Web PWA Dashboard Deployment

Launch HTTPS server locally from the `docs/` directory:
```bash
cd docs
python3 -c "
import http.server, ssl
server_address = ('0.0.0.0', 8443)
httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain(certfile='../cert.pem', keyfile='../key.pem')
httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
print('HTTPS server running on https://localhost:8443')
httpd.serve_forever()
"
```
Open `https://<YOUR-IP>:8443` in Chrome / Safari / Edge and tap **"Add to Home Screen"** or **"Install App"** to use offline.

---

## 📁 Repository Structure

```
.
├── .gitignore          # Rules ignoring build output & SSL certs
├── README.md           # System architecture, pinout tables, & build guide
├── boards/             # Board definitions
├── docs/               # GitHub Pages Web PWA Dashboard
│   ├── index.html      # Web Bluetooth Dashboard (Offline UI & Leaflet Maps)
│   ├── manifest.json   # PWA App Manifest
│   └── sw.js           # Service Worker for 100% Offline Caching
├── platformio.ini      # PlatformIO build configuration for both environments
└── src/
    ├── main.cpp        # RAK4631 Beacon C++ Firmware (SX1262 LoRa + BLE + MOSFET Gate)
    └── tx_main.cpp     # Heltec V3 C++ Firmware (OLED UI, Signal Analyzer, Telemetry)
```