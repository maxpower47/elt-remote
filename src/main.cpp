#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "protocol_binary.h"
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrf.h>

using namespace Adafruit_LittleFS_Namespace;

#define HW_PIN_WB_IO2       (32 + 2)   // P1.02 — 3V3_S power rail
#define HW_LORA_CS          (32 + 10)  // P1.10 — NSS/CS
#define HW_LORA_DIO1        (32 + 15)  // P1.15 — DIO1 interrupt
#define HW_LORA_RST         (32 + 6)   // P1.06 — RESET
#define HW_LORA_BUSY        (32 + 14)  // P1.14 — BUSY
#define HW_LORA_ANT_PWR     (32 + 5)   // P1.05 — RF switch
#define HW_LORA_SCK         (32 + 11)  // P1.11 — SPI SCK
#define HW_LORA_MOSI        (32 + 12)  // P1.12 — SPI MOSI
#define HW_LORA_MISO        (32 + 13)  // P1.13 — SPI MISO
#define HW_LED_GREEN        (32 + 3)   // P1.03 — RF Activity LED
#define HW_LED_BLUE         (32 + 4)   // P1.04 — Armed / Active Status LED
#define HW_MOSFET_TRIG_PIN  20         // P0.20 — RAK19003 J7 Header "TX" Pad (3.3V Logic)

#undef PIN_VBAT
#define PIN_VBAT            A1         // D15 / P0.05 / AIN3 — Hardware Battery Divider on RAK Baseboard

#define STATE_FILE_PATH     "/beacon_state.bin"

struct BeaconNVMData {
    uint8_t state;
    uint32_t timerDurationSec;
    uint32_t remainingSecAtSave;
    uint32_t runDurationSec;
    uint32_t remainingRunSecAtSave;
    uint32_t magic;
};
#define NVM_MAGIC 0xDEADBEEF

BLEUart bleuart;

class RAKRadioLibHal : public ArduinoHal {
public:
    RAKRadioLibHal() : ArduinoHal(SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0)) {}

    void init() override {
        this->spi->begin();
        NRF_SPIM3->ENABLE = 0;

        NRF_SPIM3->PSEL.SCK  = HW_LORA_SCK;
        NRF_SPIM3->PSEL.MOSI = HW_LORA_MOSI;
        NRF_SPIM3->PSEL.MISO = HW_LORA_MISO;

        nrf_gpio_cfg(HW_LORA_SCK,  NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_cfg(HW_LORA_MOSI, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
                     NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_cfg(HW_LORA_MISO, NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);

        NRF_SPIM3->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
    }

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (mode == OUTPUT) {
            nrf_gpio_cfg_output(pin);
        } else if (mode == INPUT_PULLUP) {
            nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
        } else {
            nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);
        }
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (value) nrf_gpio_pin_set(pin);
        else       nrf_gpio_pin_clear(pin);
    }

    uint32_t digitalRead(uint32_t pin) override {
        return nrf_gpio_pin_read(pin);
    }

    void attachInterrupt(uint32_t hwPin, void (*func)(void), uint32_t mode) override {
        uint32_t featherPin = hwToFeather(hwPin);
        if (featherPin != 0xFFFFFFFFu) {
            ::attachInterrupt(featherPin, func, (int)mode);
        }
    }

    void detachInterrupt(uint32_t hwPin) override {
        uint32_t featherPin = hwToFeather(hwPin);
        if (featherPin != 0xFFFFFFFFu) {
            ::detachInterrupt(featherPin);
        }
    }

private:
    static uint32_t hwToFeather(uint32_t hwPin) {
        extern const uint32_t g_ADigitalPinMap[];
        for (uint32_t i = 0; i < 35; i++) {
            if (g_ADigitalPinMap[i] == hwPin) return i;
        }
        return 0xFFFFFFFFu;
    }
};

RAKRadioLibHal hal;
SX1262 radio = new Module(&hal, HW_LORA_CS, HW_LORA_DIO1, HW_LORA_RST, HW_LORA_BUSY);

enum SystemState { DISARMED = 0, ARMED_TIMER = 1, ACTIVE = 2 };
SystemState currentState = DISARMED;

uint32_t armTimerStartMs = 0;
uint32_t armTimerDurationSec = 0;
uint32_t runTimerStartMs = 0;
uint32_t runTimerDurationSec = 0;
uint32_t remainingSeconds = 0;

float batteryVoltage = 4.10;
double lastLat = 0.0;
double lastLon = 0.0;
bool gpsFixValid = false;

void ledSet(uint32_t hwPin, bool on) {
    if (on) nrf_gpio_pin_set(hwPin);
    else     nrf_gpio_pin_clear(hwPin);
}

void blinkCode(uint32_t hwPin, int count, int ms) {
    for (int i = 0; i < count; i++) {
        nrf_gpio_pin_set(hwPin);   delay(ms);
        nrf_gpio_pin_clear(hwPin); delay(ms);
    }
}

// Control MOSFET Gate Driver Pin & Blue Status LED
void updateHardwareOutputs() {
    if (currentState == ACTIVE) {
        // ACTIVE ALARM: MOSFET Gate HIGH (Power Siren ON), Blue LED Solid ON
        nrf_gpio_pin_set(HW_MOSFET_TRIG_PIN);
        ledSet(HW_LED_BLUE, true);
    } else if (currentState == ARMED_TIMER) {
        // ARMED COUNTDOWN: MOSFET Gate LOW (Siren OFF), Blue LED slow pulse
        nrf_gpio_pin_clear(HW_MOSFET_TRIG_PIN);
        static uint32_t lastPulse = 0;
        static uint32_t ledPulseStart = 0;
        static bool ledPulseActive = false;
        if (millis() - lastPulse > 1000) {
            lastPulse = millis();
            ledPulseActive = true;
            ledPulseStart = millis();
            ledSet(HW_LED_BLUE, true);
        }
        if (ledPulseActive && millis() - ledPulseStart >= 40) {
            ledSet(HW_LED_BLUE, false);
            ledPulseActive = false;
        }
    } else {
        // DISARMED: MOSFET Gate LOW, Blue LED OFF
        nrf_gpio_pin_clear(HW_MOSFET_TRIG_PIN);
        ledSet(HW_LED_BLUE, false);
    }
}

void saveStateToNVM() {
    File file = InternalFS.open(STATE_FILE_PATH, FILE_O_WRITE);
    if (file) {
        BeaconNVMData nvm;
        nvm.state = (uint8_t)currentState;
        nvm.timerDurationSec = armTimerDurationSec;
        nvm.remainingSecAtSave = (currentState == ARMED_TIMER) ? remainingSeconds : 0;
        nvm.runDurationSec = runTimerDurationSec;
        nvm.remainingRunSecAtSave = (currentState == ACTIVE && runTimerDurationSec > 0) ? remainingSeconds : 0;
        nvm.magic = NVM_MAGIC;
        
        file.seek(0);
        file.write((uint8_t*)&nvm, sizeof(BeaconNVMData));
        file.close();
    }
}

void loadStateFromNVM() {
    if (!InternalFS.exists(STATE_FILE_PATH)) return;

    File file = InternalFS.open(STATE_FILE_PATH, FILE_O_READ);
    if (file) {
        BeaconNVMData nvm;
        if (file.read((uint8_t*)&nvm, sizeof(BeaconNVMData)) == sizeof(BeaconNVMData)) {
            if (nvm.magic == NVM_MAGIC) {
                currentState = (SystemState)nvm.state;
                armTimerDurationSec = nvm.timerDurationSec;
                runTimerDurationSec = nvm.runDurationSec;
                
                if (currentState == ARMED_TIMER) {
                    armTimerStartMs = millis();
                    armTimerDurationSec = nvm.remainingSecAtSave > 0 ? nvm.remainingSecAtSave : armTimerDurationSec;
                    remainingSeconds = armTimerDurationSec;
                } else if (currentState == ACTIVE) {
                    if (runTimerDurationSec > 0) {
                        runTimerStartMs = millis();
                        runTimerDurationSec = nvm.remainingRunSecAtSave > 0 ? nvm.remainingRunSecAtSave : runTimerDurationSec;
                        remainingSeconds = runTimerDurationSec;
                    } else {
                        remainingSeconds = 0;
                    }
                    updateHardwareOutputs();
                } else {
                    remainingSeconds = 0;
                }
            }
        }
        file.close();
    }
}

float readBatteryVoltage() {
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);

    // Discard initial sample to settle SAADC
    analogRead(PIN_VBAT);
    delayMicroseconds(50);

    uint32_t rawSum = 0;
    for (int i = 0; i < 8; i++) {
        rawSum += analogRead(PIN_VBAT);
        delayMicroseconds(50);
    }
    float rawAvg = (float)rawSum / 8.0F;
    // 3.0V reference / 4096 * 1.73 divider compensation
    float vbat = (rawAvg * 3.0F / 4096.0F) * 1.73F;
    if (vbat < 2.0F) return 0.0F; // Below 2.0V = USB power / no battery
    return vbat;
}

void updateTimerState() {
    if (currentState == ARMED_TIMER) {
        uint32_t elapsedSec = (millis() - armTimerStartMs) / 1000;
        if (elapsedSec >= armTimerDurationSec) {
            // Delay expired -> Transition to ACTIVE
            currentState = ACTIVE;
            if (runTimerDurationSec > 0) {
                runTimerStartMs = millis();
                remainingSeconds = runTimerDurationSec;
            } else {
                remainingSeconds = 0;
            }
            saveStateToNVM();
        } else {
            remainingSeconds = armTimerDurationSec - elapsedSec;
        }
    } else if (currentState == ACTIVE) {
        if (runTimerDurationSec > 0) {
            uint32_t elapsedSec = (millis() - runTimerStartMs) / 1000;
            if (elapsedSec >= runTimerDurationSec) {
                // Run time expired -> Auto-disarm
                currentState = DISARMED;
                remainingSeconds = 0;
                runTimerDurationSec = 0;
                saveStateToNVM();
                Serial.println("[RAK4631] Run Timer Expired -> Auto-DISARM");
            } else {
                remainingSeconds = runTimerDurationSec - elapsedSec;
            }
        } else {
            remainingSeconds = 0;
        }
    } else {
        remainingSeconds = 0;
    }

    updateHardwareOutputs();
}

void onDio1();
void broadcastBleTelemetry();
extern volatile bool rxFlag;
extern volatile bool txDoneFlag;
extern volatile bool txInProgress;
extern uint32_t txStartMs;
void executeCommand(uint8_t cmdId, uint32_t delaySec, uint32_t runSec);
void startAsyncTelemetryTx(const LoRaTelemetryPacket &pkt);

void parseCommand(String jsonStr) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (err) return;

    if (doc.containsKey("cmd")) {
        String cmd = doc["cmd"].as<String>();
        if (cmd == "DISARM") {
            executeCommand(CMD_DISARM, 0, 0);
        } else if (cmd == "ARM_NOW") {
            uint32_t runSec = doc["run_sec"] | 0;
            executeCommand(CMD_ARM_NOW, 0, runSec);
        } else if (cmd == "ARM_TIMER") {
            uint32_t delaySec = doc["delay_sec"] | doc["sec"] | 0;
            uint32_t runSec = doc["run_sec"] | 0;
            executeCommand(CMD_ARM_TIMER, delaySec, runSec);
        }
    }
}

uint8_t txSequenceNumber = 0;

// Shared state for deferred async ACK TX (set by executeCommand, consumed by loop)
static LoRaTelemetryPacket pendingTxPkt;
static bool pendingTxQueued = false;
static uint32_t pendingAckAt = 0;

void generateBinaryTelemetry(LoRaTelemetryPacket &pkt) {
    updateTimerState();
    batteryVoltage = readBatteryVoltage();
    bool usbPower = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;

    memset(&pkt, 0, sizeof(LoRaTelemetryPacket));
    pkt.msg_type = MSG_TYPE_TELEMETRY;
    pkt.seq_num = ++txSequenceNumber;
    
    switch (currentState) {
        case DISARMED:    pkt.state = STATE_ID_DISARMED; break;
        case ARMED_TIMER: pkt.state = STATE_ID_ARMED_TIMER; break;
        case ACTIVE:      pkt.state = STATE_ID_ACTIVE; break;
    }

    pkt.flags = 0;
    if (usbPower) pkt.flags |= FLAG_USB_POWER;
    if (gpsFixValid) pkt.flags |= FLAG_GPS_VALID;

    pkt.batt_mv = (uint16_t)(batteryVoltage * 1000.0F);
    pkt.remaining_sec = remainingSeconds;
    pkt.lat_e7 = (int32_t)(lastLat * 1e7);
    pkt.lon_e7 = (int32_t)(lastLon * 1e7);
}

void recoverRadio() {
    Serial.println("[RAK4631] Watchdog: Recovering SX1262 Radio...");
    nrf_gpio_pin_clear(HW_LORA_RST); delay(20);
    nrf_gpio_pin_set(HW_LORA_RST);   delay(100);

    int state = radio.begin(915.0, 125.0, 11, 8, 0x34, 22, 16, 1.8, false);
    if (state == RADIOLIB_ERR_NONE) {
        radio.setDio2AsRfSwitch(true);
        uint8_t sw[] = {0x34, 0x44};
        radio.setSyncWord(sw, 2);
        radio.autoLDRO();
        radio.setDio1Action(onDio1);
        radio.startReceive();
        Serial.println("[RAK4631] Watchdog: Radio recovered successfully!");
    } else {
        Serial.printf("[RAK4631] Watchdog: Radio recovery failed (%d)\n", state);
    }
}

void executeCommand(uint8_t cmdId, uint32_t delaySec, uint32_t runSec) {
    if (cmdId == CMD_DISARM) {
        currentState = DISARMED;
        remainingSeconds = 0;
        armTimerDurationSec = 0;
        runTimerDurationSec = 0;
        updateHardwareOutputs();
        saveStateToNVM();
        Serial.println("[RAK4631] Command Received: DISARM (MOSFET OFF)");
    } else if (cmdId == CMD_ARM_NOW) {
        currentState = ACTIVE;
        armTimerDurationSec = 0;
        runTimerDurationSec = runSec;
        if (runTimerDurationSec > 0) {
            runTimerStartMs = millis();
            remainingSeconds = runTimerDurationSec;
            Serial.printf("[RAK4631] Command Received: ARM_NOW with %ds runtime\n", runTimerDurationSec);
        } else {
            remainingSeconds = 0;
            Serial.println("[RAK4631] Command Received: ARM_NOW (indefinite)");
        }
        updateHardwareOutputs();
        saveStateToNVM();
    } else if (cmdId == CMD_ARM_TIMER) {
        runTimerDurationSec = runSec;
        if (delaySec > 0) {
            currentState = ARMED_TIMER;
            armTimerDurationSec = delaySec;
            armTimerStartMs = millis();
            remainingSeconds = armTimerDurationSec;
            Serial.printf("[RAK4631] Command Received: ARM_TIMER (delay: %ds, run: %ds)\n", armTimerDurationSec, runTimerDurationSec);
        } else {
            // Delay is 0 -> Arm immediately with the specified run duration!
            currentState = ACTIVE;
            armTimerDurationSec = 0;
            if (runTimerDurationSec > 0) {
                runTimerStartMs = millis();
                remainingSeconds = runTimerDurationSec;
            } else {
                remainingSeconds = 0;
            }
            Serial.printf("[RAK4631] Command Received: ARM_TIMER (immediate run: %ds)\n", runTimerDurationSec);
        }
        updateHardwareOutputs();
        saveStateToNVM();
    }

    // Transmit immediate BLE telemetry update if BLE connected
    if (Bluefruit.connected()) {
        broadcastBleTelemetry();
    }

    // Queue non-blocking LoRa ACK reply (150ms turnaround for SF11 preamble settle)
    // The startAsyncTelemetryTx call is deferred slightly so the remote's RX mode
    // has time to settle after completing its own TX.
    pendingAckAt = millis() + 150;
    pendingTxQueued = true;
    generateBinaryTelemetry(pendingTxPkt);
}

String generateTelemetry() {
    updateTimerState();
    batteryVoltage = readBatteryVoltage();
    bool usbPower = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;

    StaticJsonDocument<512> doc;
    doc["type"] = "TELEMETRY";
    doc["device"] = "RAK4631_BEACON";
    switch (currentState) {
        case DISARMED:    doc["state"] = "DISARMED"; break;
        case ARMED_TIMER: doc["state"] = "ARMED_TIMER"; doc["remaining_sec"] = remainingSeconds; break;
        case ACTIVE:      doc["state"] = "ACTIVE"; break;
    }
    doc["batt"] = usbPower ? 0.0F : batteryVoltage;
    doc["usb"] = usbPower;
    if (usbPower && batteryVoltage >= 2.5F) {
        doc["raw_batt"] = batteryVoltage;
    }
    JsonObject gps = doc.createNestedObject("gps");
    gps["lat"] = lastLat; gps["lon"] = lastLon; gps["valid"] = gpsFixValid;
    String out; serializeJson(doc, out); return out;
}

void broadcastBleTelemetry() {
    if (Bluefruit.connected()) {
        String payload = generateTelemetry() + "\n";
        int len = payload.length();
        int offset = 0;
        while (offset < len) {
            int chunkSize = min(20, len - offset);
            bleuart.write((const uint8_t*)(payload.c_str() + offset), chunkSize);
            bleuart.flush();
            offset += chunkSize;
            delay(15);
        }
    }
}

void connect_callback(uint16_t conn_handle) {
    (void) conn_handle;
    broadcastBleTelemetry();
}

String bleRxBufferRAK = "";
volatile bool bleDataAvailable = false;

void bleRxCallback(uint16_t conn_handle) {
    (void) conn_handle;
    bleDataAvailable = true;
}

volatile bool rxFlag = false;
volatile bool txDoneFlag = false;
volatile bool txInProgress = false;
uint32_t txStartMs = 0;
#define TX_TIMEOUT_MS 2000  // If TX takes longer than 2s, the radio has locked up

void onDio1() {
    if (txInProgress) {
        txDoneFlag = true;
    } else {
        rxFlag = true;
    }
}

// Non-blocking start of a LoRa transmission. Sets txInProgress = true.
// The loop() polls txDoneFlag (set by onDio1 ISR) and txStartMs for timeout.
void startAsyncTelemetryTx(const LoRaTelemetryPacket &pkt) {
    txInProgress = true;
    txDoneFlag = false;
    txStartMs = millis();
    radio.clearDio1Action();
    int res = radio.startTransmit((uint8_t*)&pkt, sizeof(pkt));
    radio.setDio1Action(onDio1);
    if (res != RADIOLIB_ERR_NONE) {
        Serial.printf("[RAK4631] startTransmit failed: %d -> recovering\n", res);
        txInProgress = false;
        recoverRadio();
    }
}

void setup() {
    nrf_gpio_cfg_output(HW_LED_GREEN); ledSet(HW_LED_GREEN, false);
    nrf_gpio_cfg_output(HW_LED_BLUE);  ledSet(HW_LED_BLUE, false);
    nrf_gpio_cfg_output(HW_MOSFET_TRIG_PIN); nrf_gpio_pin_clear(HW_MOSFET_TRIG_PIN);

    nrf_gpio_cfg_output(HW_PIN_WB_IO2);   nrf_gpio_pin_set(HW_PIN_WB_IO2);
    nrf_gpio_cfg_output(HW_LORA_ANT_PWR); nrf_gpio_pin_set(HW_LORA_ANT_PWR);
    delay(200);

    nrf_gpio_cfg_output(HW_LORA_CS);   nrf_gpio_pin_set(HW_LORA_CS);
    nrf_gpio_cfg_input(HW_LORA_BUSY, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(HW_LORA_RST);
    nrf_gpio_pin_clear(HW_LORA_RST); delay(20);
    nrf_gpio_pin_set(HW_LORA_RST);   delay(100);

    Serial.begin(115200);

    InternalFS.begin();
    loadStateFromNVM();

    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("ELT Beacon");
    Bluefruit.Periph.setConnectCallback(connect_callback);

    bleuart.begin();
    bleuart.setRxCallback(bleRxCallback);

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleuart);

    // Put name in Scan Response to prevent 31-byte primary payload truncation
    Bluefruit.ScanResponse.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    int state = radio.begin(915.0, 125.0, 11, 8, 0x34, 22, 16, 1.8, false);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[RAK4631] RadioLib OK (SF11 / CR8 / 16-Sym Preamble / 22dBm)");
        radio.setDio2AsRfSwitch(true);
        uint8_t sw[] = {0x34, 0x44};
        radio.setSyncWord(sw, 2);
        radio.autoLDRO();
        radio.setDio1Action(onDio1);
        radio.startReceive();
        blinkCode(HW_LED_GREEN, 3, 300);
    } else {
        Serial.printf("[RAK4631] FAIL: %d\n", state);
        int n = (abs(state) > 20) ? 5 : abs(state);
        while (true) { blinkCode(HW_LED_BLUE, n, 150); delay(1000); }
    }
}

void loop() {
    updateTimerState();
    updateHardwareOutputs();

    // --- TX Done / TX Timeout handler ---
    if (txInProgress) {
        if (txDoneFlag) {
            // TX completed successfully via DIO1 IRQ
            txDoneFlag = false;
            txInProgress = false;
            rxFlag = false;
            radio.finishTransmit();
            int rxState = radio.startReceive();
            if (rxState != RADIOLIB_ERR_NONE) {
                recoverRadio();
            }
            ledSet(HW_LED_GREEN, false);
            if (Bluefruit.connected()) {
                broadcastBleTelemetry();
            }
            Serial.println("[RAK4631] TX Done (async)");
        } else if (millis() - txStartMs > TX_TIMEOUT_MS) {
            // TX hung — radio is locked up, recover it
            Serial.println("[RAK4631] TX TIMEOUT -> recovering radio");
            txInProgress = false;
            txDoneFlag = false;
            recoverRadio();
            ledSet(HW_LED_GREEN, false);
        }
        // While TX is in progress, skip RX and BLE processing to avoid SPI contention
        return;
    }

    // --- Deferred ACK TX from executeCommand ---
    if (pendingTxQueued && millis() >= pendingAckAt) {
        pendingTxQueued = false;
        ledSet(HW_LED_GREEN, true);
        startAsyncTelemetryTx(pendingTxPkt);
        return;
    }

    // --- BLE RX ---
    if (bleDataAvailable || bleuart.available()) {
        bleDataAvailable = false;
        while (bleuart.available()) {
            char c = (char) bleuart.read();
            bleRxBufferRAK += c;
        }

        while (true) {
            int firstBrace = bleRxBufferRAK.indexOf('{');
            if (firstBrace > 0) {
                bleRxBufferRAK = bleRxBufferRAK.substring(firstBrace);
                firstBrace = 0;
            } else if (firstBrace < 0) {
                if (bleRxBufferRAK.length() > 0) bleRxBufferRAK = "";
                break;
            }

            int openCount = 0;
            int endIdx = -1;
            for (unsigned int i = 0; i < bleRxBufferRAK.length(); i++) {
                if (bleRxBufferRAK[i] == '{') openCount++;
                else if (bleRxBufferRAK[i] == '}') {
                    openCount--;
                    if (openCount == 0) {
                        endIdx = i;
                        break;
                    }
                }
            }

            if (endIdx > 0) {
                String input = bleRxBufferRAK.substring(0, endIdx + 1);
                bleRxBufferRAK = bleRxBufferRAK.substring(endIdx + 1);
                Serial.println("[BLE RAK4631 RX] Executing Command: " + input);
                parseCommand(input);
            } else {
                if (bleRxBufferRAK.length() > 256) {
                    bleRxBufferRAK = "";
                }
                break;
            }
        }
    }

    // --- IRQ safety net: poll SX1262 IRQ register every 500ms ---
    static uint32_t lastIrqCheck = 0;
    if (millis() - lastIrqCheck > 500) {
        lastIrqCheck = millis();
        uint16_t irq = radio.getIrqFlags();
        if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
            rxFlag = true;
        } else if (irq & (RADIOLIB_SX126X_IRQ_CRC_ERR | RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_TIMEOUT)) {
            radio.finishReceive();
            radio.startReceive();
        }
    }

    // --- Serial command input (debug) ---
    if (Serial.available()) {
        String sCmd = Serial.readStringUntil('\n');
        sCmd.trim();
        if (sCmd == "ARM_NOW") {
            executeCommand(CMD_ARM_NOW, 0, 0);
        } else if (sCmd == "DISARM") {
            executeCommand(CMD_DISARM, 0, 0);
        } else if (sCmd.startsWith("ARM_TIMER")) {
            uint32_t delaySec = 3600;
            uint32_t runSec = 0;
            int spaceIdx = sCmd.indexOf(' ');
            if (spaceIdx > 0) {
                String rest = sCmd.substring(spaceIdx + 1);
                int space2 = rest.indexOf(' ');
                if (space2 > 0) {
                    delaySec = rest.substring(0, space2).toInt();
                    runSec = rest.substring(space2 + 1).toInt();
                } else {
                    delaySec = rest.toInt();
                }
            }
            executeCommand(CMD_ARM_TIMER, delaySec, runSec);
        }
    }

    // --- LoRa RX ---
    if (rxFlag) {
        rxFlag = false;
        uint8_t rxBuffer[256];
        memset(rxBuffer, 0, sizeof(rxBuffer));
        size_t len = radio.getPacketLength();
        if (len > sizeof(rxBuffer)) len = sizeof(rxBuffer);
        int state = radio.readData(rxBuffer, len);
        float rssi = radio.getRSSI();
        float snr = radio.getSNR();

        int rxState = radio.startReceive();
        if (rxState != RADIOLIB_ERR_NONE) {
            recoverRadio();
        }

        Serial.printf("[RAK4631 LoRa RX] State: %d, Len: %d, RSSI: %.1f dBm, SNR: %.1f dB\n", state, len, rssi, snr);

        if (state == RADIOLIB_ERR_NONE && len > 0) {
            if (len >= sizeof(LoRaCommandPacket) && rxBuffer[0] == MSG_TYPE_COMMAND) {
                LoRaCommandPacket *cmdPkt = (LoRaCommandPacket*)rxBuffer;
                Serial.printf("[RX Binary Cmd] Type: 0x%02X, Cmd: 0x%02X, Delay: %u, Run: %u\n", cmdPkt->msg_type, cmdPkt->cmd, cmdPkt->delay_sec, cmdPkt->runtime_sec);
                executeCommand(cmdPkt->cmd, cmdPkt->delay_sec, cmdPkt->runtime_sec);
            } else if (rxBuffer[0] == '{') {
                String str = "";
                for (size_t i = 0; i < len; i++) str += (char)rxBuffer[i];
                Serial.println("[RX JSON Cmd] " + str);
                parseCommand(str);
            }
        }
    }

    // --- Periodic telemetry TX every 10s ---
    static uint32_t lastTx = 0;
    if (millis() - lastTx > 10000) {
        lastTx = millis();
        ledSet(HW_LED_GREEN, true);
        LoRaTelemetryPacket pkt;
        generateBinaryTelemetry(pkt);
        startAsyncTelemetryTx(pkt);
        Serial.printf("[RAK4631] Queued async TX | Seq: %u, State: %u, Batt: %u mV\n", pkt.seq_num, pkt.state, pkt.batt_mv);
    }

    delay(10);
}
