#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <esp_task_wdt.h>
#include "freertos/queue.h"
#include "protocol_binary.h"

#define PIN_BUTTON   0
#define PIN_VEXT     36
#define PIN_OLED_RST 21

// Heltec V3 Battery ADC: voltage divider output on GPIO 1, enable switch on GPIO 37
#define PIN_VBAT_ADC 1
#define PIN_ADC_CTRL 37

#define LORA_CS      8
#define LORA_DIO1    14
#define LORA_RST     12
#define LORA_BUSY    13

// BLE Service & Characteristic UUIDs (Nordic UART Service - NUS)
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
static SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64, I2C_ONE, 100000);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

#define BLE_CMD_MAX_LEN 128
QueueHandle_t bleCommandQueue = NULL;
String lastTxStatus = "Ready";

// Telemetry & State Data
String rawJson = "";
String beaconState = "UNKNOWN";
uint32_t remainingSec = 0;
float beaconBatt = 0.0;
float txBatt = 0.0; // Heltec V3 Local Battery Voltage
float beaconLat = 0.0;
float beaconLon = 0.0;
bool gpsValid = false;

float lastRssi = 0.0;
float lastSnr = 0.0;
uint32_t packetCount = 0;
uint32_t lastRxTime = 0;

int currentScreen = 0;
const int TOTAL_SCREENS = 4;

const char* menuItems[] = {
    "1. ARM IMMEDIATELY",
    "2. ARM WITH TIMER >",
    "3. DISARM BEACON"
};
const int TOTAL_MENU_ITEMS = 3;
int selectedMenuItem = 0;

int menuMode = 0;
int selectedHours = 1;

// Precise Battery & USB Power Detection for Heltec V3
float readTxBatteryVoltage() {
    analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
    pinMode(PIN_ADC_CTRL, OUTPUT);
    digitalWrite(PIN_ADC_CTRL, HIGH); // Enable battery voltage divider
    delay(5);

    uint32_t mvSum = 0;
    for (int i = 0; i < 8; i++) {
        mvSum += analogReadMilliVolts(PIN_VBAT_ADC);
        delay(1);
    }
    digitalWrite(PIN_ADC_CTRL, LOW); // Turn OFF divider to save battery

    float pinMvAvg = (float)mvSum / 8.0F;
    float vbat = (pinMvAvg * 4.9F) / 1000.0F;

    if (vbat > 4.06F || vbat < 2.5F) {
        return 0.0F; // 0.0V indicates USB Power
    }
    return vbat;
}

volatile bool rxFlag = false;
volatile bool txDoneFlag = false;
volatile bool txInProgress = false;
uint32_t txStartMs = 0;
#define TX_TIMEOUT_MS 2000

void IRAM_ATTR onDio1() {
    if (txInProgress) {
        txDoneFlag = true;
    } else {
        rxFlag = true;
    }
}

uint8_t heltecTxSeq = 0;
LoRaCommandPacket lastSentPacket;

void recoverRadio() {
    Serial.println("[Heltec V3] Watchdog: Recovering SX1262 Radio...");
    digitalWrite(LORA_RST, LOW); delay(20);
    digitalWrite(LORA_RST, HIGH); delay(100);

    int state = radio.begin(915.0, 125.0, 11, 8, 0x34, 22, 16, 1.6, false);
    if (state == RADIOLIB_ERR_NONE) {
        radio.setDio2AsRfSwitch(true);
        uint8_t syncWordBytes[] = {0x34, 0x44};
        radio.setSyncWord(syncWordBytes, 2);
        radio.autoLDRO();
        radio.setDio1Action(onDio1);
        radio.startReceive();
        Serial.println("[Heltec V3] Watchdog: Radio recovered successfully!");
    } else {
        Serial.printf("[Heltec V3] Watchdog: Radio recovery failed (%d)\n", state);
    }
}

// Non-blocking command transmit using startTransmit().
// Returns immediately; TX completion is signalled via txDoneFlag in loop().
void transmitLoRaCommand(String cmd) {
    LoRaCommandPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.msg_type = MSG_TYPE_COMMAND;
    pkt.seq_num = ++heltecTxSeq;

    String cmdStr = cmd;
    StaticJsonDocument<128> doc;
    DeserializationError jsonErr = deserializeJson(doc, cmd);
    if (jsonErr == DeserializationError::Ok && doc.containsKey("cmd")) {
        cmdStr = doc["cmd"].as<String>();
    }

    if (cmdStr == "DISARM") {
        pkt.cmd = CMD_DISARM;
    } else if (cmdStr == "ARM_NOW") {
        pkt.cmd = CMD_ARM_NOW;
    } else if (cmdStr == "ARM_TIMER") {
        pkt.cmd = CMD_ARM_TIMER;
        if (jsonErr == DeserializationError::Ok && doc.containsKey("sec")) {
            pkt.param = doc["sec"].as<uint32_t>();
        } else {
            pkt.param = selectedHours * 3600;
        }
    }

    lastSentPacket = pkt;
    txInProgress = true;
    txDoneFlag = false;
    txStartMs = millis();
    radio.clearDio1Action();
    int res = radio.startTransmit((uint8_t*)&pkt, sizeof(pkt));
    radio.setDio1Action(onDio1);
    Serial.printf("[Heltec V3 Binary TX] Cmd: 0x%02X, Param: %u (startTransmit: %d)\n", pkt.cmd, pkt.param, res);
    if (res != RADIOLIB_ERR_NONE) {
        txInProgress = false;
        recoverRadio();
    }
}

void broadcastBleTelemetry();
void parseTelemetry(const String& jsonStr);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        broadcastBleTelemetry();
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};



class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string val = pCharacteristic->getValue();
        String input = String(val.c_str());
        input.trim();
        Serial.println("[NimBLE onWrite] Received: " + input);
        
        static String bleAccumBuffer;
        bleAccumBuffer += String(val.c_str());
        
        int depth = 0;
        for (int i = 0; i < bleAccumBuffer.length(); i++) {
            if (bleAccumBuffer[i] == '{') depth++;
            else if (bleAccumBuffer[i] == '}') {
                depth--;
                if (depth == 0) {
                    String complete = bleAccumBuffer.substring(0, i + 1);
                    bleAccumBuffer = bleAccumBuffer.substring(i + 1);
                    char cmdBuf[BLE_CMD_MAX_LEN];
                    memset(cmdBuf, 0, sizeof(cmdBuf));
                    strncpy(cmdBuf, complete.c_str(), BLE_CMD_MAX_LEN - 1);
                    xQueueSend(bleCommandQueue, cmdBuf, 0);
                    lastTxStatus = "BLE Cmd Received";
                    break;
                }
            }
        }
        if (bleAccumBuffer.length() > 256) bleAccumBuffer = "";
    }
};

String popupToastMessage = "";
uint32_t popupToastUntilMs = 0;
bool buttonWasPressedGlob = false;
uint32_t buttonPressStartGlob = 0;
bool longPressTriggeredGlob = false;

void triggerToastPopup(const String& msg) {
    popupToastMessage = msg;
    popupToastUntilMs = millis() + 2500; // Display for 2.5s
}

volatile uint32_t isrPressDownMs = 0;
volatile uint32_t isrPressUpMs = 0;
volatile uint32_t isrPressCount = 0;

void IRAM_ATTR onButtonIsr() {
    int state = digitalRead(PIN_BUTTON);
    uint32_t now = millis();
    if (state == LOW) {
        // Falling edge: Button pressed down
        if (isrPressDownMs == 0) {
            isrPressDownMs = now;
        }
        buttonWasPressedGlob = true;
    } else {
        // Rising edge: Button released up
        isrPressUpMs = now;
        isrPressCount++;
        buttonWasPressedGlob = false;
    }
}

void handleButton() {
    static uint32_t lastProcessedPressCount = 0;
    static uint32_t lastProcessedPressTime = 0;
    static bool longPressHandled = false;
    uint32_t now = millis();

    int pinLevel = digitalRead(PIN_BUTTON);

    // 1. Check for Active Long Press (While button is still held down)
    if (pinLevel == LOW && isrPressDownMs > 0 && !longPressHandled) {
        if (now - isrPressDownMs >= 650) {
            longPressHandled = true;

            if (currentScreen == 2) {
                if (menuMode == 1) {
                    uint32_t sec = (uint32_t)selectedHours * 3600;
                    String c = "{\"cmd\":\"ARM_TIMER\",\"sec\":" + String(sec) + "}";
                    char cBuf[BLE_CMD_MAX_LEN]; memset(cBuf, 0, sizeof(cBuf)); strncpy(cBuf, c.c_str(), BLE_CMD_MAX_LEN - 1); xQueueSend(bleCommandQueue, cBuf, 0);
                    lastTxStatus = "Sending " + String(selectedHours) + "h Timer...";
                    triggerToastPopup("✓ " + String(selectedHours) + "h TIMER SENT!");
                    menuMode = 0;
                    selectedMenuItem = 0;
                    currentScreen = 0;
                } else {
                    switch (selectedMenuItem) {
                        case 0:
                            {
                                String c = "{\"cmd\":\"ARM_NOW\"}";
                                char cBuf[BLE_CMD_MAX_LEN]; memset(cBuf, 0, sizeof(cBuf)); strncpy(cBuf, c.c_str(), BLE_CMD_MAX_LEN - 1); xQueueSend(bleCommandQueue, cBuf, 0);
                                lastTxStatus = "Sending Arm Now...";
                                triggerToastPopup("✓ COMMAND SENT!");
                                selectedMenuItem = 0;
                                currentScreen = 0;
                            }
                            break;
                        case 1:
                            menuMode = 1;
                            break;
                        case 2:
                            {
                                String c = "{\"cmd\":\"DISARM\"}";
                                char cBuf[BLE_CMD_MAX_LEN]; memset(cBuf, 0, sizeof(cBuf)); strncpy(cBuf, c.c_str(), BLE_CMD_MAX_LEN - 1); xQueueSend(bleCommandQueue, cBuf, 0);
                                lastTxStatus = "Sending Disarm...";
                                triggerToastPopup("✓ DISARM SENT!");
                                selectedMenuItem = 0;
                                currentScreen = 0;
                            }
                            break;
                    }
                }
            } else {
                currentScreen = 2; // Jump directly to Command menu
                menuMode = 0;
            }
        }
    }

    // 2. Check for Short Click (Triggered on Release)
    if (isrPressCount != lastProcessedPressCount) {
        lastProcessedPressCount = isrPressCount;

        uint32_t pressStart = isrPressDownMs;
        uint32_t pressEnd = isrPressUpMs;

        // Reset press start now that release occurred
        isrPressDownMs = 0;

        if (pressEnd >= pressStart && (pressEnd - lastProcessedPressTime > 120)) {
            uint32_t holdDuration = pressEnd - pressStart;

            if (!longPressHandled && holdDuration >= 10 && holdDuration < 650) {
                lastProcessedPressTime = pressEnd;

                // SHORT CLICK
                if (currentScreen == 2) {
                    if (menuMode == 1) {
                        selectedHours++;
                        if (selectedHours > 72) selectedHours = 1;
                    } else {
                        selectedMenuItem++;
                        if (selectedMenuItem >= TOTAL_MENU_ITEMS) {
                            selectedMenuItem = 0;
                            currentScreen = 3; // Advance to Screen 4 (Signal Analyzer)
                        }
                    }
                } else {
                    currentScreen = (currentScreen + 1) % TOTAL_SCREENS;
                }
            }
        }
    }

    if (pinLevel == HIGH) {
        longPressHandled = false;
        buttonWasPressedGlob = false;
    }
}

#define RSSI_HISTORY_SIZE 30
int rssiHistory[RSSI_HISTORY_SIZE];
int rssiHistoryIdx = 0;

void addRssiSample(int rssi) {
    rssiHistory[rssiHistoryIdx] = rssi;
    rssiHistoryIdx = (rssiHistoryIdx + 1) % RSSI_HISTORY_SIZE;
}

void broadcastBleTelemetry() {
    if (deviceConnected && pTxCharacteristic) {
        StaticJsonDocument<512> bleDoc;
        bleDoc["type"] = "TELEMETRY";
        bleDoc["device"] = "HELTEC_V3_TX";
        bleDoc["state"] = (beaconState.length() > 0) ? beaconState : "DISARMED";
        bleDoc["batt"] = beaconBatt;
        bleDoc["tx_batt"] = txBatt;
        bleDoc["remaining_sec"] = remainingSec;
        bleDoc["rssi"] = (int)lastRssi;
        bleDoc["snr"] = (int)lastSnr;
        bleDoc["pkts"] = packetCount;
        JsonObject bleGps = bleDoc.createNestedObject("gps");
        bleGps["lat"] = beaconLat;
        bleGps["lon"] = beaconLon;
        bleGps["valid"] = gpsValid;

        String bleJson;
        serializeJson(bleDoc, bleJson);
        bleJson += "\n";

        int len = bleJson.length();
        int offset = 0;
        while (offset < len && deviceConnected) {
            int chunkSize = min(20, len - offset);
            pTxCharacteristic->notify((const uint8_t*)(bleJson.c_str() + offset), chunkSize);
            offset += chunkSize;
            delay(20);
        }
    }
}

void parseBinaryTelemetry(const LoRaTelemetryPacket &pkt) {
    switch (pkt.state) {
        case STATE_ID_DISARMED:    beaconState = "DISARMED"; break;
        case STATE_ID_ARMED_TIMER: beaconState = "ARMED_TIMER"; break;
        case STATE_ID_ACTIVE:      beaconState = "ACTIVE"; break;
    }

    remainingSec = pkt.remaining_sec;
    bool usbPwr = (pkt.flags & FLAG_USB_POWER) != 0;
    beaconBatt = usbPwr ? 0.0F : ((float)pkt.batt_mv / 1000.0F);
    
    gpsValid = (pkt.flags & FLAG_GPS_VALID) != 0;
    beaconLat = (float)pkt.lat_e7 / 1e7;
    beaconLon = (float)pkt.lon_e7 / 1e7;

    lastRxTime = millis();
    packetCount++;
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    addRssiSample((int)lastRssi);
    txBatt = readTxBatteryVoltage();

    broadcastBleTelemetry();
}

void parseTelemetry(const String& jsonStr) {
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (err) return;

    if (doc.containsKey("state")) {
        String st = doc["state"].as<String>();
        if (st.length() > 0) {
            beaconState = st;
        }
    }
    if (doc.containsKey("remaining_sec")) remainingSec = doc["remaining_sec"].as<uint32_t>();
    if (doc.containsKey("batt")) beaconBatt = doc["batt"].as<float>();
    
    if (doc.containsKey("gps")) {
        JsonObject gps = doc["gps"];
        beaconLat = gps["lat"] | 0.0;
        beaconLon = gps["lon"] | 0.0;
        gpsValid = gps["valid"] | false;
    }

    lastRxTime = millis();
    packetCount++;
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    addRssiSample((int)lastRssi);
    txBatt = readTxBatteryVoltage();

    broadcastBleTelemetry();
}

// Custom 10x10 Pixel Art Tab Icons
void drawTabIcon(int x, int y, int iconType) {
    switch (iconType) {
        case 0: // HOME / SUMMARY ICON (Pitched roof + house body)
            display.drawLine(x + 5, y, x, y + 5);
            display.drawLine(x + 5, y, x + 10, y + 5);
            display.drawRect(x + 2, y + 5, 7, 5);
            break;
        case 1: // GPS LOCATION PIN ICON (Target pin + circle)
            display.drawCircle(x + 5, y + 3, 3);
            display.drawLine(x + 5, y + 6, x + 5, y + 10);
            break;
        case 2: // COMMAND GEAR / CONTROLS ICON (Gear teeth + center hole)
            display.drawCircle(x + 5, y + 5, 4);
            display.setPixel(x + 5, y + 5);
            display.setPixel(x + 5, y + 0); display.setPixel(x + 5, y + 10);
            display.setPixel(x + 0, y + 5); display.setPixel(x + 10, y + 5);
            break;
        case 3: // SIGNAL ANALYZER ICON (Radio Tower / Signal Waves 📶)
            // Center Antenna Pole & Node
            display.drawLine(x + 5, y + 4, x + 5, y + 10);
            display.setPixel(x + 5, y + 4);
            // Inner Signal Arcs
            display.drawLine(x + 3, y + 2, x + 3, y + 6);
            display.drawLine(x + 7, y + 2, x + 7, y + 6);
            // Outer Signal Arcs
            display.drawLine(x + 1, y + 0, x + 1, y + 8);
            display.drawLine(x + 9, y + 0, x + 9, y + 8);
            break;
    }
}

// Draw Phone-Style Battery Icon with Pixel Lightning Bolt when Charging / USB
void drawBatteryIcon(int x, int y, float vbat) {
    // Battery Body Outer Shell: 15px wide x 8px high
    display.drawRect(x, y, 15, 8);
    // Battery Positive Terminal Nub: 2px wide x 4px high
    display.fillRect(x + 15, y + 2, 2, 4);

    if (vbat < 1.0F) {
        // USB Power / Charging: Draw Pixel-Art Lightning Bolt ⚡ inside battery
        display.drawLine(x + 8, y + 1, x + 5, y + 4);
        display.drawLine(x + 5, y + 4, x + 9, y + 4);
        display.drawLine(x + 9, y + 4, x + 6, y + 7);
    } else {
        // LiPo Battery Fill Level — calibrated to single-cell LiPo discharge curve
        // 4.20V = 100%, 3.85V ≈ 60%, 3.70V ≈ 30%, 3.50V ≈ 5%, 3.30V = 0%
        int fillWidth = 0;
        if (vbat >= 3.85F) fillWidth = 11;       // Full  (>60%)
        else if (vbat >= 3.70F) fillWidth = 7;   // Mid   (30-60%)
        else if (vbat >= 3.50F) fillWidth = 4;   // Low   (5-30%)
        else fillWidth = 1;                      // Critical (<5%)

        if (fillWidth > 0) {
            display.fillRect(x + 2, y + 2, fillWidth, 4);
        }
    }
}

// Draw Header Bar with Pixel Icons & Battery Gauge
void drawHeader(const char* title) {
    const int iconX[] = { 4, 28, 52, 76 };

    for (int i = 0; i < TOTAL_SCREENS; i++) {
        int x = iconX[i];
        if (i == currentScreen) {
            // Active Tab: Highlighted Filled Background Box with Inverted Icon
            display.setColor(WHITE);
            display.fillRect(x - 3, 0, 18, 12);
            display.setColor(BLACK);
            drawTabIcon(x, 1, i);
            display.setColor(WHITE);
        } else {
            // Inactive Tab: Outline Icon
            display.setColor(WHITE);
            drawTabIcon(x, 1, i);
        }
    }

    // Phone-style battery indicator in top right corner
    drawBatteryIcon(108, 2, txBatt);

    // Separator line
    display.drawHorizontalLine(0, 13, 128);
}

String formatTime(uint32_t totalSec) {
    uint32_t hrs = totalSec / 3600;
    uint32_t mins = (totalSec % 3600) / 60;
    uint32_t secs = totalSec % 60;
    char buf[16];
    if (hrs > 0) {
        snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", hrs, mins, secs);
    } else {
        snprintf(buf, sizeof(buf), "%lu:%02lu", mins, secs);
    }
    return String(buf);
}

int voltageToPercent(float v) {
    if (v <= 3.3F) return 0;
    if (v >= 4.2F) return 100;
    if (v >= 4.0F) return 80 + (int)(((v - 4.0F) / 0.2F) * 20.0F);
    if (v >= 3.8F) return 55 + (int)(((v - 3.8F) / 0.2F) * 25.0F);
    if (v >= 3.7F) return 35 + (int)(((v - 3.7F) / 0.1F) * 20.0F);
    if (v >= 3.6F) return 15 + (int)(((v - 3.6F) / 0.1F) * 20.0F);
    return (int)(((v - 3.3F) / 0.3F) * 15.0F);
}

void renderScreen0() {
    drawHeader("1. SYSTEM SUMMARY");

    uint32_t ageSec = (lastRxTime > 0) ? (millis() - lastRxTime) / 1000 : 0;
    bool isLost = (lastRxTime > 0 && ageSec >= 35);
    String displayState = isLost ? "LOST LINK" : beaconState;

    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 15, "State:");
    display.setFont(ArialMT_Plain_16);
    display.drawString(38, 13, displayState);

    display.setFont(ArialMT_Plain_10);
    if (isLost) {
        display.drawString(0, 31, "Bcn Batt: UNKNOWN (Stale)");
    } else if (beaconState == "ARMED_TIMER") {
        display.drawString(0, 31, "Timer: " + formatTime(remainingSec) + " remaining");
    } else {
        if (lastRxTime > 0) {
            String battStr;
            if (beaconBatt > 1.0F) {
                battStr = String(voltageToPercent(beaconBatt)) + "% (" + String(beaconBatt, 2) + "V)";
            } else {
                battStr = "USB Power";
            }
            display.drawString(0, 31, "Bcn Batt: " + battStr);
        } else {
            display.drawString(0, 31, "Bcn Batt: N/A");
        }
    }

    if (lastRxTime > 0) {
        if (isLost) {
            display.drawString(0, 47, "LINK LOST (" + String(ageSec) + "s ago)");
        } else {
            display.drawString(0, 47, "Rx: " + String(ageSec) + "s ago | RSSI: " + String((int)lastRssi) + "dBm");
        }
    } else {
        display.drawString(0, 47, "Waiting for beacon...");
    }
}

// Convert Decimal Degrees (e.g. 34.0522, -118.2437) to Degrees Minutes Seconds (DMS) Format
String decimalToDMS(float val, bool isLat) {
    char dir = isLat ? (val >= 0 ? 'N' : 'S') : (val >= 0 ? 'E' : 'W');
    val = abs(val);

    int deg = (int)val;
    float remainderMins = (val - deg) * 60.0F;
    int mins = (int)remainderMins;
    float secs = (remainderMins - mins) * 60.0F;

    char buf[32];
    snprintf(buf, sizeof(buf), "%d°%02d'%04.1f\"%c", deg, mins, secs, dir);
    return String(buf);
}


// Convert Latitude & Longitude to 3-Level CAP Cell Grid Identifier (e.g. 4086ABC)
String getCapCellGrid(float lat, float lon) {
    if (lat == 0.0F && lon == 0.0F) return "--";
    float aLat = abs(lat);
    float aLon = abs(lon);

    int baseLat = (int)aLat;
    int baseLon = (int)aLon;

    char base[16];
    if (baseLon >= 100) {
        snprintf(base, sizeof(base), "%02d%d", baseLat, baseLon);
    } else {
        snprintf(base, sizeof(base), "%02d%02d", baseLat, baseLon);
    }

    float latMin = (aLat - (float)baseLat) * 60.0F;
    float lonMin = (aLon - (float)baseLon) * 60.0F;

    auto getQuad = [](float lMin, float loMin, float span) -> char {
        bool isNorth = fmod(lMin, span) >= (span / 2.0F);
        bool isWest = fmod(loMin, span) >= (span / 2.0F);
        if (isNorth && isWest) return 'A';  // NW
        if (isNorth && !isWest) return 'B'; // NE
        if (!isNorth && isWest) return 'C'; // SW
        return 'D';                         // SE
    };

    char q1 = getQuad(latMin, lonMin, 60.0F);
    char q2 = getQuad(latMin, lonMin, 30.0F);
    char q3 = getQuad(latMin, lonMin, 15.0F);

    char result[32];
    snprintf(result, sizeof(result), "%s%c%c%c", base, q1, q2, q3);
    return String(result);
}

void renderScreen1() {
    drawHeader("2. GPS TELEMETRY");

    uint32_t ageSec = (lastRxTime > 0) ? (millis() - lastRxTime) / 1000 : 0;
    bool isLost = (lastRxTime > 0 && ageSec >= 35);

    display.setFont(ArialMT_Plain_10);
    if (lastRxTime > 0 && gpsValid) {
        // Line 1: CAP Grid and Fix status
        display.drawString(0, 15, "CAP: " + getCapCellGrid(beaconLat, beaconLon) + "  [3D FIX]");
        
        // Line 2 & 3: DMS Coordinates
        display.drawString(0, 27, "Lat: " + decimalToDMS(beaconLat, true));
        display.drawString(0, 39, "Lon: " + decimalToDMS(beaconLon, false));
        
        // Line 4: Raw Coordinates
        display.drawString(0, 51, "Raw: " + String(beaconLat, 5) + ", " + String(beaconLon, 5));
    } else {
        display.drawString(0, 15, "CAP: --");
        display.drawString(0, 27, "Lat: 0°00'00.0\"N");
        display.drawString(0, 39, "Lon: 0°00'00.0\"W");
        display.drawString(0, 51, isLost ? "LINK LOST (STALE)" : "No GPS Fix");
    }
}

void renderScreen2() {
    if (menuMode == 1) {
        drawHeader("SET TIMER DURATION");

        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 14, "Set Arm Delay (Hours):");

        display.setFont(ArialMT_Plain_24);
        String hrsStr = String(selectedHours) + (selectedHours == 1 ? " Hour" : " Hours");
        display.drawString(10, 26, hrsStr);

        display.setFont(ArialMT_Plain_10);
        display.drawHorizontalLine(0, 50, 128);
        display.drawString(0, 52, "[Click: +1h | Hold: Confirm]");
    } else {
        drawHeader("3. COMMAND MENU");

        display.setFont(ArialMT_Plain_10);
        for (int i = 0; i < TOTAL_MENU_ITEMS; i++) {
            int y = 14 + (i * 9);
            if (i == selectedMenuItem) {
                display.drawString(0, y, "> " + String(menuItems[i]));
            } else {
                display.drawString(0, y, "  " + String(menuItems[i]));
            }
        }

        display.drawHorizontalLine(0, 52, 128);
        display.drawString(0, 53, "[Click: Next | Hold: Select]");
    }
}

int getLinkHealthPercentage(int rssi, float snr, bool isLost) {
    if (isLost || lastRxTime == 0) return 0;
    // RSSI range: -120 dBm (0%) to -50 dBm (100%)
    int pct = map(constrain(rssi, -120, -50), -120, -50, 0, 100);
    return pct;
}

void renderScreen3() {
    drawHeader("4. SIGNAL ANALYZER");

    uint32_t ageSec = (lastRxTime > 0) ? (millis() - lastRxTime) / 1000 : 0;
    bool isLost = (lastRxTime > 0 && ageSec >= 30);

    // On link loss, periodically append a drop (-130 dBm) to sparkline history
    static uint32_t lastDropSample = 0;
    if (isLost && (millis() - lastDropSample > 3000)) {
        lastDropSample = millis();
        addRssiSample(-130);
    }

    int linkPct = isLost ? 0 : getLinkHealthPercentage((int)lastRssi, lastSnr, false);

    display.setFont(ArialMT_Plain_10);
    
    // Top Metrics Row
    if (isLost) {
        display.drawString(0, 15, "RSSI: STALE");
        display.drawString(64, 15, "SNR: --");
        display.drawString(0, 27, "Link: 0% (LOST LINK " + String(ageSec) + "s)");
    } else if (lastRxTime > 0) {
        display.drawString(0, 15, "RSSI:" + String((int)lastRssi) + "dBm");
        display.drawString(64, 15, "SNR:" + String(lastSnr, 1) + "dB");
        display.drawString(0, 27, "Link:" + String(linkPct) + "% (HEALTHY)");
    } else {
        display.drawString(0, 15, "RSSI: -- dBm");
        display.drawString(64, 15, "SNR: -- dB");
        display.drawString(0, 27, "Link Health: Searching...");
    }

    // Sparkline Graph Box (Width: 120px, Height: 22px, y=39)
    int graphX = 4;
    int graphY = 39;
    int graphW = 120;
    int graphH = 22;

    display.drawRect(graphX, graphY, graphW, graphH);

    // Draw RSSI Sparkline Trend Line
    for (int i = 0; i < RSSI_HISTORY_SIZE - 1; i++) {
        int idx1 = (rssiHistoryIdx + i) % RSSI_HISTORY_SIZE;
        int idx2 = (rssiHistoryIdx + i + 1) % RSSI_HISTORY_SIZE;

        int r1 = rssiHistory[idx1];
        int r2 = rssiHistory[idx2];

        if (r1 != 0 && r2 != 0) {
            int y1 = map(constrain(r1, -130, -50), -130, -50, graphY + graphH - 2, graphY + 2);
            int y2 = map(constrain(r2, -130, -50), -130, -50, graphY + graphH - 2, graphY + 2);

            int x1 = graphX + 2 + (i * 4);
            int x2 = graphX + 2 + ((i + 1) * 4);

            display.drawLine(x1, y1, x2, y2);
        }
    }
}

// Global tracking for button hold progress bar
extern bool buttonWasPressedGlob;
extern uint32_t buttonPressStartGlob;
extern bool longPressTriggeredGlob;

void renderToastOverlay() {
    // 1. Live Hold Progress Bar (Renders at bottom of screen ONLY on Command Menu screen 2)
    if (currentScreen == 2 && buttonWasPressedGlob && !longPressTriggeredGlob && isrPressDownMs > 0) {
        uint32_t holdMs = millis() - isrPressDownMs;
        if (holdMs > 150) {
            int fillW = map(constrain((int)holdMs, 0, 650), 0, 650, 0, 104);
            
            // Draw progress bar outline & fill (x=12, y=53, w=104, h=8)
            display.setColor(BLACK);
            display.fillRect(0, 52, 128, 12); // Clear footer area
            display.setColor(WHITE);
            display.drawRect(12, 53, 104, 8);
            if (fillW > 0) {
                display.fillRect(12, 53, fillW, 8);
            }
        }
    }

    // 2. Pop-Up Modal Toast Box (Renders when command is triggered)
    if (millis() < popupToastUntilMs && popupToastMessage.length() > 0) {
        display.setColor(BLACK);
        display.fillRect(6, 18, 116, 30);
        display.setColor(WHITE);
        display.drawRect(6, 18, 116, 30);
        display.drawRect(7, 19, 114, 28); // Double border outline

        display.setFont(ArialMT_Plain_10);
        display.drawString(14, 26, popupToastMessage);
    }
}

void broadcastBleTelemetry();
void updateOLED() {
    display.clear();
    switch (currentScreen) {
        case 0: renderScreen0(); break;
        case 1: renderScreen1(); break;
        case 2: renderScreen2(); break;
        case 3: renderScreen3(); break;
    }
    renderToastOverlay();
    display.display();
}

void setup() {
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, LOW);
    delay(100);

    pinMode(PIN_OLED_RST, OUTPUT);
    digitalWrite(PIN_OLED_RST, LOW);
    delay(50);
    digitalWrite(PIN_OLED_RST, HIGH);
    delay(100);

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButtonIsr, CHANGE);
    Serial.begin(115200);

    bleCommandQueue = xQueueCreate(4, BLE_CMD_MAX_LEN);

    txBatt = readTxBatteryVoltage();

    Wire.begin(SDA_OLED, SCL_OLED);
    Wire.setClock(100000); // 100kHz standard mode: maximum noise immunity on battery
    Wire.setTimeOut(25);   // 25ms hardware timeout: prevents I2C bus hang from blocking CPU

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    updateOLED();

    BLEDevice::init("ELT Remote");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
                        );

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
                                       );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    // Hardware Task Watchdog (5-second timeout)
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);

    int state = radio.begin(915.0, 125.0, 11, 8, 0x34, 22, 16, 1.6, false);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[Heltec V3 RadioLib] Init SUCCESS! (SF11 / CR8 / 16-Sym Preamble / 22dBm)");
        radio.setDio2AsRfSwitch(true);
        uint8_t syncWordBytes[] = {0x34, 0x44};
        radio.setSyncWord(syncWordBytes, 2);
        radio.autoLDRO();
        radio.setDio1Action(onDio1);
        radio.startReceive();
    } else {
        Serial.printf("[Heltec V3 RadioLib] Init FAIL, code: %d\n", state);
    }
}

String activeCommand = "";
uint32_t activeCommandStart = 0;
int activeCommandRetries = 0;

void loop() {
    esp_task_wdt_reset(); // Feed the hardware watchdog: if the CPU ever hangs >5s, hardware auto-reboots
    handleButton();

    // --- TX Done / TX Timeout handler (runs first to free SPI bus ASAP) ---
    if (txInProgress) {
        if (txDoneFlag) {
            txDoneFlag = false;
            txInProgress = false;
            rxFlag = false;
            radio.finishTransmit();
            int rxState = radio.startReceive();
            if (rxState != RADIOLIB_ERR_NONE) {
                recoverRadio();
            }
            Serial.println("[Heltec V3] TX Done (async)");
        } else if (millis() - txStartMs > TX_TIMEOUT_MS) {
            Serial.println("[Heltec V3] TX TIMEOUT -> recovering radio");
            txInProgress = false;
            txDoneFlag = false;
            activeCommand = "";
            lastTxStatus = "TX Timeout";
            recoverRadio();
        }
        // While TX in progress, skip everything else to avoid SPI contention
        // Still update OLED so display stays responsive
        static uint32_t lastOledDuringTx = 0;
        if (millis() - lastOledDuringTx > 100) {
            lastOledDuringTx = millis();
            updateOLED();
        }
        delay(5);
        return;
    }

    // --- Serial command input (debug) ---
    // NOTE: Must be non-blocking! On battery, the USB-UART bridge (CH9102) is
    // unpowered and GPIO 44 (RX) floats, picking up noise that triggers
    // Serial.available(). readStringUntil('\n') would block up to 1000ms.
    {
        static String serialAccum;
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                serialAccum.trim();
                if (serialAccum.length() > 0) {
                    char cBuf[BLE_CMD_MAX_LEN];
                    memset(cBuf, 0, sizeof(cBuf));
                    if (serialAccum == "ARM_NOW") {
                        snprintf(cBuf, sizeof(cBuf), "{\"cmd\":\"ARM_NOW\"}");
                    } else if (serialAccum == "DISARM") {
                        snprintf(cBuf, sizeof(cBuf), "{\"cmd\":\"DISARM\"}");
                    } else if (serialAccum.startsWith("ARM_TIMER")) {
                        uint32_t sec = 3600;
                        int spaceIdx = serialAccum.indexOf(' ');
                        if (spaceIdx > 0) sec = serialAccum.substring(spaceIdx + 1).toInt();
                        snprintf(cBuf, sizeof(cBuf), "{\"cmd\":\"ARM_TIMER\",\"sec\":%u}", sec);
                    } else {
                        strncpy(cBuf, serialAccum.c_str(), sizeof(cBuf) - 1);
                    }
                    xQueueSend(bleCommandQueue, cBuf, 0);
                }
                serialAccum = "";
            } else {
                serialAccum += c;
                if (serialAccum.length() > 128) serialAccum = ""; // overflow protection
            }
        }
    }

    // --- Send pending commands from BLE queue ---
    bool sentNewCmd = false;
    char cmdBuf[BLE_CMD_MAX_LEN];
    while (xQueueReceive(bleCommandQueue, cmdBuf, 0) == pdTRUE) {
        activeCommand = String(cmdBuf);
        activeCommandStart = millis();
        activeCommandRetries = 0;
        lastTxStatus = activeCommand;
        transmitLoRaCommand(activeCommand);
        sentNewCmd = true;
    }

    // --- Non-blocking retry if no telemetry ACK received within 2500ms ---
    if (!sentNewCmd && !txInProgress && activeCommand.length() > 0 &&
        (millis() - activeCommandStart > 2500)) {
        activeCommandRetries++;
        if (activeCommandRetries <= 3) {
            activeCommandStart = millis();
            Serial.printf("[Heltec V3 Retry %d] Re-sending %s\n", activeCommandRetries, activeCommand.c_str());
            transmitLoRaCommand(activeCommand);
        } else {
            activeCommand = "";
            lastTxStatus = "TX Timeout";
        }
    }

    // --- LoRa RX (Hardware ISR Driven) ---
    if (rxFlag) {
        rxFlag = false;
        uint8_t rxBuffer[256];
        memset(rxBuffer, 0, sizeof(rxBuffer));
        size_t len = radio.getPacketLength();
        if (len > sizeof(rxBuffer)) len = sizeof(rxBuffer);
        int state = radio.readData(rxBuffer, len);

        int rxState = radio.startReceive();
        if (rxState != RADIOLIB_ERR_NONE) {
            recoverRadio();
        }

        if (state == RADIOLIB_ERR_NONE && len > 0) {
            if (len >= sizeof(LoRaTelemetryPacket) && rxBuffer[0] == MSG_TYPE_TELEMETRY) {
                LoRaTelemetryPacket *telPkt = (LoRaTelemetryPacket*)rxBuffer;

                if (activeCommand.length() > 0) {
                    activeCommand = "";
                    if (lastSentPacket.cmd == CMD_DISARM) {
                        lastTxStatus = "DISARMED OK";
                    } else if (lastSentPacket.cmd == CMD_ARM_NOW) {
                        lastTxStatus = "ARMED OK";
                    } else if (lastSentPacket.cmd == CMD_ARM_TIMER) {
                        lastTxStatus = "TIMER SET OK";
                    }
                }

                parseBinaryTelemetry(*telPkt);
                Serial.printf("[Heltec V3 Binary RX] Seq: %u, State: %u, Batt: %u mV\n", telPkt->seq_num, telPkt->state, telPkt->batt_mv);
            } else if (rxBuffer[0] == '{') {
                activeCommand = "";
                String str = "";
                for (size_t i = 0; i < len; i++) str += (char)rxBuffer[i];
                rawJson = str;
                parseTelemetry(str);
                Serial.println("[Heltec V3 JSON RX] " + str);
            }
        }
    }

    // --- OLED display update (Event-Driven + 1Hz periodic timer tick) ---
    static uint32_t lastOledUpdate = 0;
    static int lastScreen = -1;
    static int lastMenuItem = -1;
    static int lastHours = -1;
    static uint32_t lastRxCount = 0;
    static bool lastBtnState = false;

    bool forceUpdate = false;
    if (currentScreen != lastScreen || selectedMenuItem != lastMenuItem || selectedHours != lastHours || packetCount != lastRxCount || buttonWasPressedGlob != lastBtnState) {
        forceUpdate = true;
        lastScreen = currentScreen;
        lastMenuItem = selectedMenuItem;
        lastHours = selectedHours;
        lastRxCount = packetCount;
        lastBtnState = buttonWasPressedGlob;
    }

    // Refresh immediately on user interaction / packet rx, otherwise 1000ms (1Hz) to tick the seconds counter
    uint32_t refreshInterval = (buttonWasPressedGlob || millis() < popupToastUntilMs) ? 50 : 1000;

    if (forceUpdate || (millis() - lastOledUpdate >= refreshInterval)) {
        updateOLED();
        lastOledUpdate = millis();
    }

    delay(10);
}
