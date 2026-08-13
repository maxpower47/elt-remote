#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <SSD1306Wire.h>

#define PIN_BUTTON   0
#define PIN_VEXT     36
#define PIN_OLED_RST 21

#define LORA_CS      8
#define LORA_DIO1    14
#define LORA_RST     12
#define LORA_BUSY    13

// Heltec V3 Battery Hardware Pins
#define PIN_VBAT_ADC 1   // GPIO 1 / ADC1_CH0
#define PIN_ADC_CTRL 37  // Power switch for battery divider

// BLE Service & Characteristic UUIDs (Nordic UART Service - NUS)
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
static SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

String pendingCommandToSend = "";
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

volatile bool rxFlag = false;
void IRAM_ATTR onDio1() {
    rxFlag = true;
}

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

void transmitLoRaCommand(String cmd) {
    radio.clearDio1Action();
    int txRes = radio.transmit(cmd);
    Serial.printf("[Heltec V3 TX] %s (code: %d)\n", cmd.c_str(), txRes);
    rxFlag = false;
    radio.setDio1Action(onDio1);
    radio.startReceive();
}

class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string val = pCharacteristic->getValue();
        String input = String(val.c_str());
        input.trim();
        Serial.println("[NimBLE onWrite] Received: " + input);
        if (input.startsWith("{")) {
            pendingCommandToSend = input;
            lastTxStatus = "BLE Cmd Received";
        }
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

void handleButton() {
    int btnState = digitalRead(PIN_BUTTON);
    uint32_t now = millis();

    if (btnState == LOW) {
        if (!buttonWasPressedGlob) {
            buttonWasPressedGlob = true;
            buttonPressStartGlob = now;
            longPressTriggeredGlob = false;
        } else if (!longPressTriggeredGlob && (now - buttonPressStartGlob >= 1000)) {
            // TRIGGER INSTANTLY AT 1000ms HOLD THRESHOLD (WHILE STILL HELD DOWN)
            longPressTriggeredGlob = true;

            if (currentScreen == 2) {
                if (menuMode == 1) {
                    uint32_t sec = (uint32_t)selectedHours * 3600;
                    pendingCommandToSend = "{\"cmd\":\"ARM_TIMER\",\"sec\":" + String(sec) + "}";
                    lastTxStatus = "Sending " + String(selectedHours) + "h Timer...";
                    triggerToastPopup("✓ " + String(selectedHours) + "h TIMER SENT!");
                    menuMode = 0;
                    selectedMenuItem = 0;
                    currentScreen = 0;
                } else {
                    switch (selectedMenuItem) {
                        case 0:
                            pendingCommandToSend = "{\"cmd\":\"ARM_NOW\"}";
                            lastTxStatus = "Sending Arm Now...";
                            triggerToastPopup("✓ COMMAND SENT!");
                            selectedMenuItem = 0;
                            currentScreen = 0;
                            break;
                        case 1:
                            menuMode = 1;
                            break;
                        case 2:
                            pendingCommandToSend = "{\"cmd\":\"DISARM\"}";
                            lastTxStatus = "Sending Disarm...";
                            triggerToastPopup("✓ DISARM SENT!");
                            selectedMenuItem = 0;
                            currentScreen = 0;
                            break;
                    }
                }
            } else {
                currentScreen = 2; // Jump directly to Command menu
                menuMode = 0;
            }
        }
    } else if (btnState == HIGH && buttonWasPressedGlob) {
        uint32_t pressDuration = now - buttonPressStartGlob;
        buttonWasPressedGlob = false;

        // SHORT CLICK (only if long press wasn't already triggered while held down)
        if (!longPressTriggeredGlob && pressDuration > 50) {
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
        longPressTriggeredGlob = false;
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
        // LiPo Battery Fill Level (0 to 3 bars)
        int fillWidth = 0;
        if (vbat >= 3.95F) fillWidth = 11;       // Full (3 bars)
        else if (vbat >= 3.75F) fillWidth = 7;   // Medium (2 bars)
        else if (vbat >= 3.55F) fillWidth = 4;   // Low (1 bar)
        else fillWidth = 1;                      // Empty (0 bars)

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

void renderScreen0() {
    drawHeader("1. SYSTEM SUMMARY");

    uint32_t ageSec = (lastRxTime > 0) ? (millis() - lastRxTime) / 1000 : 0;
    bool isLost = (lastRxTime > 0 && ageSec >= 30);
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
            String battStr = (beaconBatt > 1.0F) ? (String(beaconBatt, 2) + "V") : "USB Power";
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

// Calculate Haversine Distance between two GPS points (in meters)
float calculateDistanceMeters(float lat1, float lon1, float lat2, float lon2) {
    if (lat1 == 0.0F || lat2 == 0.0F) return 0.0F;
    float R = 6371000.0F; // Earth radius in meters
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sin(dLat / 2.0F) * sin(dLat / 2.0F) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon / 2.0F) * sin(dLon / 2.0F);
    float c = 2.0F * atan2(sqrt(a), sqrt(1.0F - a));
    return R * c;
}

// Calculate Initial Bearing / Compass Heading (0 to 360 deg)
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    if (lat1 == 0.0F || lat2 == 0.0F) return 0.0F;
    float y = sin(radians(lon2 - lon1)) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) -
              sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(lon2 - lon1));
    float brng = degrees(atan2(y, x));
    if (brng < 0.0F) brng += 360.0F;
    return brng;
}

// Convert degrees (0-360) to 8-point Cardinal Direction
const char* getCardinalDirection(float brng) {
    const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"};
    int idx = (int)((brng + 22.5F) / 45.0F);
    return dirs[idx % 8];
}

void renderScreen1() {
    drawHeader("2. GPS TELEMETRY");

    uint32_t ageSec = (lastRxTime > 0) ? (millis() - lastRxTime) / 1000 : 0;
    bool isLost = (lastRxTime > 0 && ageSec >= 30);

    display.setFont(ArialMT_Plain_10);
    if (isLost) {
        display.drawString(0, 15, "Fix Status: STALE (LOST LINK)");
    } else {
        display.drawString(0, 15, "Fix: " + String(gpsValid ? "VALID FIX 3D" : "NO FIX / SEARCH"));
    }

    if (lastRxTime > 0 && gpsValid) {
        display.drawString(0, 27, "Lat: " + decimalToDMS(beaconLat, true));
        display.drawString(0, 39, "Lon: " + decimalToDMS(beaconLon, false));
        
        // Calculate Distance & Bearing (assuming base origin or current beacon pos)
        float distM = calculateDistanceMeters(34.0522F, -118.2437F, beaconLat, beaconLon);
        float brng = calculateBearing(34.0522F, -118.2437F, beaconLat, beaconLon);

        String distStr = (distM >= 1000.0F) ? (String(distM / 1000.0F, 2) + " km") : (String((int)distM) + " m");
        String brngStr = String((int)brng) + "° " + String(getCardinalDirection(brng));

        display.drawString(0, 51, "Dist: " + distStr + "  Hdg: " + brngStr);
    } else {
        display.drawString(0, 27, "Lat: 0°00'00.0\"N");
        display.drawString(0, 39, "Lon: 0°00'00.0\"W");
        display.drawString(0, 51, "Dist: --          Hdg: --");
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
    if (currentScreen == 2 && buttonWasPressedGlob && !longPressTriggeredGlob) {
        uint32_t holdMs = millis() - buttonPressStartGlob;
        if (holdMs > 150) {
            int fillW = map(constrain((int)holdMs, 0, 1000), 0, 1000, 0, 104);
            
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
    Serial.begin(115200);

    txBatt = readTxBatteryVoltage();

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

    int state = radio.begin(915.0, 125.0, 7, 5, 0x34, 22, 8, 1.6, false);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[Heltec V3 RadioLib] Init SUCCESS!");
        radio.setDio2AsRfSwitch(true);
        uint8_t syncWordBytes[] = {0x34, 0x44};
        radio.setSyncWord(syncWordBytes, 2);
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
    handleButton();

    // Transmit pending commands immediately
    if (pendingCommandToSend.length() > 0) {
        activeCommand = pendingCommandToSend;
        pendingCommandToSend = "";
        activeCommandStart = millis();
        activeCommandRetries = 0;
        
        lastTxStatus = activeCommand;
        transmitLoRaCommand(activeCommand);
    }
    // Asynchronous re-transmit if no telemetry ACK received within 500ms
    else if (activeCommand.length() > 0 && (millis() - activeCommandStart > 500)) {
        activeCommandRetries++;
        if (activeCommandRetries <= 3) {
            activeCommandStart = millis();
            Serial.printf("[Heltec V3 Retry %d] Re-sending %s\n", activeCommandRetries, activeCommand.c_str());
            transmitLoRaCommand(activeCommand);
        } else {
            activeCommand = ""; // Give up after 3 retries
            lastTxStatus = "TX Timeout";
        }
    }

    if (rxFlag) {
        rxFlag = false;
        String str;
        int state = radio.readData(str);
        radio.startReceive();
        if (state == RADIOLIB_ERR_NONE && str.length() > 0) {
            activeCommand = ""; // Clear pending command retry upon receiving telemetry!
            rawJson = str;
            parseTelemetry(str);
            Serial.println("[Heltec V3 RX] " + str);
        }
    }

    updateOLED();
    delay(20);
}
