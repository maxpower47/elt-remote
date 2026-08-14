# ELT-Remote Project Instructions & Guidelines

## 1. Project Overview & Architecture
`elt-remote` is a dual-device Emergency Locator Transmitter (ELT) / Search & Rescue (SAR) beacon system paired with a mobile-friendly Web Bluetooth PWA dashboard.

### Core Hardware Components:
1. **Beacon Device (Transceiver / Target)**:
   - **Board**: RAKwireless WisBlock Core **RAK4631** (Nordic nRF52840 + Semtech SX1262 LoRa).
   - **Baseboard**: **RAK19003 Mini Baseboard** (or RAK19007 / RAK5005-O).
   - **Outputs**: High-power MOSFET switch triggered via `HW_MOSFET_TRIG_PIN` (`P0.20` / J7 Header "TX" Pad) for external sirens/strobes, Status LEDs (`P1.04` Blue, `P1.03` Green).
   - **Battery Monitoring**: Voltage divider routed to **`A1` (`P0.05` / `AIN3`)** with `analogReference(AR_INTERNAL_3_0)` and 1.73x compensation factor.
   - **USB Detection**: Hardware VBUS monitoring via `NRF_POWER->USBREGSTATUS`.
   - **Bluetooth**: Advertises as **`ELT Beacon`** with Nordic UART Service (NUS).

2. **Remote Controller (Transceiver / Handheld)**:
   - **Board**: **Heltec WiFi LoRa 32 V3** (Espressif ESP32-S3 + Semtech SX1262 LoRa).
   - **Display**: Onboard 0.96" SSD1306 OLED (I2C: SDA 17, SCL 18, RST 21).
   - **Input**: User button on `GPIO 0` (Short press: navigate tabs/menus; Long press 1000ms: select/execute).
   - **Battery Monitoring**: Onboard battery ADC on `GPIO 1` (ADC1_CH0).
   - **Bluetooth**: Advertises as **`ELT Remote`** with Nordic UART Service (NUS).

3. **Web Dashboard (PWA / Mobile UI)**:
   - Located in `docs/index.html` (hosted on GitHub Pages).
   - Direct Web Bluetooth API connection to either `ELT Beacon` or `ELT Remote`.
   - Live GPS mapping with Leaflet, DMS coordinates, live bearing/distance tracking via `navigator.geolocation`, and LiPo battery percentage indicators.

---

## 2. RF & Communication Protocols

### LoRa RadioLib Parameters:
- **Frequency**: `915.0 MHz` (US ISM Band)
- **Bandwidth**: `125.0 kHz`
- **Spreading Factor**: `SF10` (High-sensitivity, long-range link margin)
- **Coding Rate**: `CR 4/6` (CR = 6)
- **Sync Word**: `{0x34, 0x44}` (Private Network)
- **Output Power**: `+22 dBm`
- **Preamble Length**: `8 symbols`
- **Beacon Telemetry Interval**: Every `10,000 ms` (10 seconds)
- **Command Turnaround Delay**: `250 ms`
- **Command Retry Engine (Heltec V3)**: Asynchronous non-blocking retries every `1200 ms` (up to 3 retries).

### Nordic UART Service (NUS) UUIDs:
- **Service UUID**: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- **RX Characteristic** (Write): `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- **TX Characteristic** (Notify): `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

### JSON Protocol:
- **Telemetry Payload**:
  ```json
  {
    "type": "TELEMETRY",
    "device": "RAK4631_BEACON",
    "state": "DISARMED",
    "batt": 4.12,
    "usb": false,
    "remaining_sec": 0,
    "gps": { "lat": 34.0522, "lon": -118.2437, "valid": true }
  }
  ```
- **Control Commands**:
  - `{"cmd":"ARM_NOW"}` $\rightarrow$ Transition to `ACTIVE` (MOSFET ON).
  - `{"cmd":"DISARM"}` $\rightarrow$ Transition to `DISARMED` (MOSFET OFF).
  - `{"cmd":"ARM_TIMER","sec":3600}` $\rightarrow$ Transition to `ARMED_TIMER` with countdown in seconds.

---

## 3. Development, Build & Upload Rules

### Automated Build & Test Script (`./build.sh`):
A unified bash build script is provided at the repository root:
- **Run all unit tests (C++ Native & JS)**:
  ```bash
  ./build.sh test
  ```
- **Build both devices**:
  ```bash
  ./build.sh all
  ```
- **Build and upload RAK4631**:
  ```bash
  ./build.sh beacon --upload
  ```
- **Build and upload Heltec V3**:
  ```bash
  ./build.sh remote --upload
  ```
- **Clean builds**:
  ```bash
  ./build.sh clean
  ```

### Direct PlatformIO Commands:
- **Run C++ Native Tests**:
  ```bash
  pio test -e native
  ```
- **Run JS Tests**:
  ```bash
  node test/dashboard_test.js
  ```
- **RAK4631**:
  ```bash
  pio run -e wiscore_rak4631 --target upload
  ```
- **Heltec WiFi LoRa 32 V3**:
  ```bash
  pio run -e heltec_wifi_lora_32_V3 --target upload
  ```

### Critical Development Rules:
1. **RadioLib DIO1 Interrupt Isolation**:
   - Always call `radio.clearDio1Action()` before calling `radio.transmit()`, and restore `radio.setDio1Action(onDio1)` + `radio.startReceive()` afterwards to prevent TX-done interrupts from corrupting receiver state.
2. **BLE UART Packet Reassembly**:
   - Web Bluetooth transmits data in 20-byte chunks.
   - Always accumulate incoming BLE bytes into a buffer and trim non-JSON characters (`indexOf('{')` to `lastIndexOf('}')`) before calling `deserializeJson` or `JSON.parse`.
3. **PWA Versioning & Caching**:
   - Whenever `docs/index.html` is modified, increment the version badge in `docs/index.html` (e.g. `v1.3.x`) and update `CACHE_NAME` in `docs/sw.js` to ensure browsers flush stale cache.
