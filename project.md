# Project Handoff: Asynchronous LoRa ELT Activation System

## 1. Project Overview

This project involves developing a MicroPython-based control system for a Civil Air Patrol (CAP) practice Emergency Locator Transmitter (ELT). The system consists of a remote-controlled ground beacon (Receiver) and a handheld command node (Transmitter). Both units communicate over 915 MHz LoRa and can be independently managed via a Web Bluetooth (BLE) application.

The architecture relies heavily on `uasyncio` for non-blocking concurrency, ensuring RF packet processing, hardware interrupts, and BLE UART streams are handled seamlessly.

## 2. Hardware Architecture

### Receiver Node (The Ground Beacon)

* **Base Board:** RAK19003 Mini Base Board
* **Core Module:** RAK4631 (Nordic nRF52840 MCU, native BLE 5.0, SX1262 LoRa)
* **Location:** RAK12501 GNSS Location Module
* **Actuator:** 3.3V Optoisolated MOSFET Switching Module (Switches a 7.5V DC load from a C-cell battery sled to the Pointer Cadet 6000 ELT)
* **Power:** 2000mAh LiPo battery (103450 footprint)
* **Timekeeping:** Internal nRF52840 hardware RTC (synced via GPS)

### Transmitter Node (The Command Dongle)

* **Hardware:** Heltec WiFi LoRa 32 V3 (ESP32-S3 MCU, SX1262 LoRa, 0.96" OLED)
* **Interface:** 1x physical programmable button (GPIO 0)
* **Power:** 3000mAh LiPo battery
* **Role:** Acts as a LoRa-to-BLE bridge and provides a standalone fallback UI for scenario directors.

---

## 3. Software & Communication Stack

* **Firmware:** MicroPython (Specific ports for nRF52840 and ESP32-S3)
* **Concurrency:** `uasyncio` event loops on both devices.
* **RF Protocol:** Custom raw LoRa packet structure (915 MHz, SX1262 driver).
* **Local Connectivity:** BLE UART service running on both nodes.
* **Client Interface:** A single HTML/JS webpage utilizing the Web Bluetooth API to connect, read, and write to the BLE UART characteristic of either device.

---

## 4. Functional Requirements

### Part A: The Receiver Logic (nRF52840)

1. **State Management:** Must maintain distinct states (`DISARMED`, `ARMED_TIMER`, `ACTIVE`).
2. **GPS Polling:** Asynchronously wake the RAK12501 module periodically (e.g., every 4 hours) to acquire a fix, sync the internal RTC, and sleep the GPS to conserve power.
3. **RF Listening:** Maintain a continuous, non-blocking LoRa listening state.
4. **Actuation:** Pull the designated GPIO pin HIGH to open the MOSFET gate and activate the ELT when a direct remote command is received or a countdown timer expires.
5. **Telemetry Broadcast:** Upon receiving an interrogation packet, transmit its current status, battery voltage, and last known GPS coordinates back over LoRa.
6. **BLE Bridge:** Accept direct configuration (timers, manual overrides) via BLE UART from a smartphone.

### Part B: The Transmitter Logic (ESP32-S3)

1. **LoRa-to-BLE Bridging:** Listen for incoming LoRa packets from the beacon and immediately push the payload strings to the connected Web Bluetooth client via BLE UART.
2. **Command Transmission:** Accept commands from the Web Bluetooth client (e.g., `ARM 120`, `ABORT`, `PING`) and broadcast them over LoRa.
3. **OLED Display Management:** Render basic status text (`[ CONNECTED ]`, `[ SEARCHING ]`, Timer values).
4. **1-Button User Interface:** Implement a non-blocking state machine using `time.ticks_ms()` to differentiate between a short click (< 500ms) and a long press (> 1000ms).

### Part C: The 1-Button UI State Machine (Transmitter)

The physical button on the Heltec V3 must handle both display cycling and timer configuration without blocking the RF listening loop.

* **Mode 0: Display Mode (Default)**
* *Short Click:* Cycles the OLED through pages (Status -> GPS Pos -> Timer Remaining -> back to Status).
* *Long Press (while on Timer page):* Transitions to Config Mode.


* **Mode 1: Config Mode (Setting the Timer)**
* *Visual Indicator:* Timer value blinks on the OLED.
* *Short Click:* Increments the countdown timer in 15-minute blocks. Wraps to 00:00 after reaching the maximum limit (e.g., 12 hours).
* *Long Press:* Saves the timer value, broadcasts the arming packet over LoRa to the beacon, and returns to Mode 0.



### Part D: The Web Bluetooth Interface (HTML/JS)

1. **Connection Handling:** A responsive webpage that prompts the user to pair with a nearby BLE device (filtering for the specific UUID of the nodes).
2. **Unified Control:** The webpage must handle connecting to *either* the Transmitter (to use it as a remote control bridge) *or* directly to the Receiver (for configuration when standing next to the ground beacon).
3. **Command Generation:** Buttons and form fields to generate the plaintext or JSON commands sent over BLE (e.g., "Activate Now", "Set Timer: 2 Hours", "Abort/Disarm").
4. **Telemetry Rendering:** A dashboard section to parse incoming BLE streams and display the beacon's armed status, countdown timer, battery level, and GPS coordinates (with a link to map the coordinates).