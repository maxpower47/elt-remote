"""
Transmitter Node Firmware for Heltec WiFi LoRa 32 V3 (ESP32-S3)
MicroPython firmware implementation
"""

import uasyncio as asyncio
import time
from machine import Pin, I2C
import ssd1306
import protocol

# Heltec V3 Hardware Pins
BUTTON_PIN = 0
OLED_SDA = 17
OLED_SCL = 18
OLED_RST = 21
VEXT_CTRL = 36

MODE_DISPLAY = 0
MODE_CONFIG = 1

PAGE_STATUS = 0
PAGE_GPS = 1
PAGE_TIMER = 2

class TransmitterNode:
    def __init__(self):
        self.mode = MODE_DISPLAY
        self.current_page = PAGE_STATUS
        self.config_timer_minutes = 0

        # Enable VEXT Power (Active LOW)
        self.vext = Pin(VEXT_CTRL, Pin.OUT)
        self.vext.value(0)
        time.sleep_ms(100)

        # Hardware Reset sequence for OLED (GPIO 21)
        self.oled_rst = Pin(OLED_RST, Pin.OUT)
        self.oled_rst.value(0)
        time.sleep_ms(50)
        self.oled_rst.value(1)
        time.sleep_ms(100)

        # Initialize I2C Bus & SSD1306
        try:
            self.i2c = I2C(0, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=400000)
            self.oled = ssd1306.SSD1306_I2C(128, 64, self.i2c)
            print("[OLED] Standard SSD1306 initialized")
        except Exception as e:
            print("[OLED Error]", e)
            self.oled = None

        # Button Pin
        self.btn = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

        # Telemetry State
        self.beacon_state = "SEARCHING"
        self.beacon_remain_sec = 0
        self.beacon_bat_v = 0.0
        self.beacon_lat = None
        self.beacon_lon = None
        self.ble_connected = False

    def update_oled(self):
        if not self.oled:
            return

        self.oled.fill(0)
        if self.mode == MODE_DISPLAY:
            if self.current_page == PAGE_STATUS:
                status_str = "[ CONNECTED ]" if self.ble_connected else "[ SEARCHING ]"
                self.oled.text(status_str, 5, 5)
                self.oled.text("State:" + str(self.beacon_state), 5, 25)
                if self.beacon_bat_v > 0:
                    self.oled.text("Bat: " + str(round(self.beacon_bat_v, 2)) + "V", 5, 45)
                else:
                    self.oled.text("No Beacon Rx", 5, 45)
            elif self.current_page == PAGE_GPS:
                self.oled.text("--- GPS FIX ---", 5, 5)
                if self.beacon_lat and self.beacon_lon:
                    self.oled.text("Lat:" + str(self.beacon_lat), 5, 25)
                    self.oled.text("Lon:" + str(self.beacon_lon), 5, 45)
                else:
                    self.oled.text("NO GPS FIX", 5, 30)
            elif self.current_page == PAGE_TIMER:
                self.oled.text("--- TIMER ---", 5, 5)
                hrs = self.beacon_remain_sec // 3600
                mins = (self.beacon_remain_sec % 3600) // 60
                self.oled.text("Rem: " + str(hrs) + "h " + str(mins) + "m", 5, 25)
                self.oled.text("LP -> Config Mode", 5, 45)
        elif self.mode == MODE_CONFIG:
            self.oled.text("* CONFIG MODE *", 5, 5)
            hrs = self.config_timer_minutes // 60
            mins = self.config_timer_minutes % 60
            self.oled.text("Timer: >" + str(hrs) + ":" + str(mins) + "<", 5, 25)
            self.oled.text("SC:+15m  LP:ARM", 5, 45)

        self.oled.show()

    def on_short_click(self):
        if self.mode == MODE_DISPLAY:
            self.current_page = (self.current_page + 1) % 3
        elif self.mode == MODE_CONFIG:
            self.config_timer_minutes = (self.config_timer_minutes + 15) % 735
        self.update_oled()

    def on_long_press(self):
        if self.mode == MODE_DISPLAY:
            if self.current_page == PAGE_TIMER:
                self.mode = MODE_CONFIG
                self.config_timer_minutes = 15
        elif self.mode == MODE_CONFIG:
            duration_sec = self.config_timer_minutes * 60
            self.mode = MODE_DISPLAY
            self.current_page = PAGE_STATUS
            self.send_lora_packet(protocol.CMD_ARM, duration_sec)
        self.update_oled()

    def send_lora_packet(self, cmd_type, duration_sec=0):
        import json
        payload = json.dumps({"cmd": cmd_type, "duration_sec": duration_sec})
        print("[RF Broadcast]", payload)

    def process_incoming_lora_packet(self, raw_str):
        print("[LoRa Inbound]", raw_str)
        cmd, data = protocol.parse_packet(raw_str)
        if data.get("type") == "TELEMETRY":
            self.beacon_state = data.get("state", "UNKNOWN")
            self.beacon_remain_sec = data.get("remain_sec", 0)
            self.beacon_bat_v = data.get("bat_v", 0.0)
            self.beacon_lat = data.get("lat")
            self.beacon_lon = data.get("lon")
            self.update_oled()

    async def task_button_polling(self):
        press_start = 0
        is_pressed = False
        while True:
            val = self.btn.value()
            if val == 0 and not is_pressed:
                is_pressed = True
                press_start = time.ticks_ms()
            elif val == 1 and is_pressed:
                is_pressed = False
                duration = time.ticks_diff(time.ticks_ms(), press_start)
                if duration < 500:
                    self.on_short_click()
                elif duration >= 1000:
                    self.on_long_press()
            await asyncio.sleep_ms(20)

    async def task_lora_interrogate_beacon(self):
        while True:
            if self.mode == MODE_DISPLAY:
                self.send_lora_packet(protocol.CMD_PING)
            await asyncio.sleep(5)

    async def run(self):
        print("[TRANSMITTER] Starting Heltec V3 Firmware...")
        self.update_oled()
        t1 = asyncio.create_task(self.task_button_polling())
        t2 = asyncio.create_task(self.task_lora_interrogate_beacon())
        await asyncio.gather(t1, t2)

if __name__ == "__main__":
    tx = TransmitterNode()
    asyncio.run(tx.run())
