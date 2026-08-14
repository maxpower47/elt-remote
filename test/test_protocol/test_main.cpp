#include <unity.h>
#include <ArduinoJson.h>
#include <cmath>
#include <string>
#include <cstring>
#include "../../src/protocol_binary.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Tested Math & Conversion Functions ---

int voltageToPercent(float v) {
    if (v <= 3.3F) return 0;
    if (v >= 4.2F) return 100;
    if (v >= 4.0F) return 80 + (int)(((v - 4.0F) / 0.2F) * 20.0F);
    if (v >= 3.8F) return 55 + (int)(((v - 3.8F) / 0.2F) * 25.0F);
    if (v >= 3.7F) return 35 + (int)(((v - 3.7F) / 0.1F) * 20.0F);
    if (v >= 3.6F) return 15 + (int)(((v - 3.6F) / 0.1F) * 20.0F);
    return (int)(((v - 3.3F) / 0.3F) * 15.0F);
}

std::string getCapCellGrid(float lat, float lon) {
    if (lat == 0.0F && lon == 0.0F) return "--";
    float aLat = std::abs(lat);
    float aLon = std::abs(lon);

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
        bool isNorth = std::fmod(lMin, span) >= (span / 2.0F);
        bool isWest = std::fmod(loMin, span) >= (span / 2.0F);
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
    return std::string(result);
}

float calculateDistanceMeters(float lat1, float lon1, float lat2, float lon2) {
    if (lat1 == 0.0F || lat2 == 0.0F) return 0.0F;
    float R = 6371000.0F;
    float dLat = (lat2 - lat1) * (M_PI / 180.0F);
    float dLon = (lon2 - lon1) * (M_PI / 180.0F);
    float a = std::sin(dLat / 2.0F) * std::sin(dLat / 2.0F) +
              std::cos(lat1 * (M_PI / 180.0F)) * std::cos(lat2 * (M_PI / 180.0F)) *
              std::sin(dLon / 2.0F) * std::sin(dLon / 2.0F);
    float c = 2.0F * std::atan2(std::sqrt(a), std::sqrt(1.0F - a));
    return R * c;
}

// --- Unit Tests ---

void test_battery_voltage_to_percent() {
    TEST_ASSERT_EQUAL_INT(100, voltageToPercent(4.20F));
    TEST_ASSERT_EQUAL_INT(100, voltageToPercent(4.35F));
    TEST_ASSERT_EQUAL_INT(80, voltageToPercent(4.00F));
    TEST_ASSERT_EQUAL_INT(55, voltageToPercent(3.80F));
    TEST_ASSERT_EQUAL_INT(35, voltageToPercent(3.70F));
    TEST_ASSERT_EQUAL_INT(15, voltageToPercent(3.60F));
    TEST_ASSERT_EQUAL_INT(0, voltageToPercent(3.30F));
    TEST_ASSERT_EQUAL_INT(0, voltageToPercent(2.50F));
}

void test_cap_cell_grid_calculation() {
    // 34.0522 N, -118.2437 W
    // Base: 34118
    // Lat: 34 deg + 3.132 min, Lon: 118 deg + 14.622 min
    // 60' span: Lat < 30 (S), Lon < 30 (E) -> D
    // 30' span: Lat < 15 (S), Lon < 15 (E) -> D
    // 15' span: Lat < 7.5 (S), Lon >= 7.5 (W) -> C
    // Result: 34118DDC
    std::string grid = getCapCellGrid(34.0522F, -118.2437F);
    TEST_ASSERT_EQUAL_STRING("34118DDC", grid.c_str());

    // 40.75 N, -86.25 W
    // Lat: 40 deg + 45 min, Lon: 86 deg + 15 min
    // 60' span: Lat >= 30 (N), Lon < 30 (E) -> B
    // 30' span: Lat(15)>=15 (N), Lon(15)>=15 (W) -> A
    // 15' span: Lat(0)<7.5 (S), Lon(0)<7.5 (E) -> D
    // Result: 4086BAD
    std::string grid2 = getCapCellGrid(40.75F, -86.25F);
    TEST_ASSERT_EQUAL_STRING("4086BAD", grid2.c_str());

    // Zero coordinates
    std::string emptyGrid = getCapCellGrid(0.0F, 0.0F);
    TEST_ASSERT_EQUAL_STRING("--", emptyGrid.c_str());
}

void test_distance_haversine() {
    // Distance between (34.0522, -118.2437) and (34.0522, -118.2437) is 0
    float d0 = calculateDistanceMeters(34.0522F, -118.2437F, 34.0522F, -118.2437F);
    TEST_ASSERT_FLOAT_WITHIN(0.1F, 0.0F, d0);

    // Distance ~ 1 degree of latitude (~111.195 km)
    float d1 = calculateDistanceMeters(34.0F, -118.0F, 35.0F, -118.0F);
    TEST_ASSERT_FLOAT_WITHIN(500.0F, 111195.0F, d1);
}

void test_json_telemetry_schema() {
    StaticJsonDocument<256> doc;
    doc["type"] = "TELEMETRY";
    doc["state"] = "ACTIVE";
    doc["batt"] = 4.12;
    doc["usb"] = false;

    std::string output;
    serializeJson(doc, output);

    StaticJsonDocument<256> parsed;
    DeserializationError err = deserializeJson(parsed, output);
    TEST_ASSERT_TRUE(err == DeserializationError::Ok);
    TEST_ASSERT_EQUAL_STRING("TELEMETRY", parsed["type"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("ACTIVE", parsed["state"].as<const char*>());
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 4.12F, parsed["batt"].as<float>());
    TEST_ASSERT_FALSE(parsed["usb"].as<bool>());
}

void test_binary_protocol_structures() {
    LoRaTelemetryPacket tel;
    memset(&tel, 0, sizeof(tel));
    tel.msg_type = MSG_TYPE_TELEMETRY;
    tel.seq_num = 42;
    tel.state = STATE_ID_ARMED_TIMER;
    tel.flags = FLAG_USB_POWER | FLAG_GPS_VALID;
    tel.batt_mv = 4150;
    tel.remaining_sec = 3600;
    tel.lat_e7 = (int32_t)(34.0522 * 1e7);
    tel.lon_e7 = (int32_t)(-118.2437 * 1e7);

    // Verify exact binary size is 18 bytes
    TEST_ASSERT_EQUAL_UINT32(18, sizeof(LoRaTelemetryPacket));

    // Simulate binary wire transfer & decode
    uint8_t wireBuffer[32];
    memcpy(wireBuffer, &tel, sizeof(LoRaTelemetryPacket));

    LoRaTelemetryPacket *decoded = (LoRaTelemetryPacket*)wireBuffer;
    TEST_ASSERT_EQUAL_UINT8(MSG_TYPE_TELEMETRY, decoded->msg_type);
    TEST_ASSERT_EQUAL_UINT8(42, decoded->seq_num);
    TEST_ASSERT_EQUAL_UINT8(STATE_ID_ARMED_TIMER, decoded->state);
    TEST_ASSERT_EQUAL_UINT8(FLAG_USB_POWER | FLAG_GPS_VALID, decoded->flags);
    TEST_ASSERT_EQUAL_UINT16(4150, decoded->batt_mv);
    TEST_ASSERT_EQUAL_UINT32(3600, decoded->remaining_sec);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 34.0522F, (float)decoded->lat_e7 / 1e7);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, -118.2437F, (float)decoded->lon_e7 / 1e7);

    // Command packet size check
    TEST_ASSERT_EQUAL_UINT32(8, sizeof(LoRaCommandPacket));
    LoRaCommandPacket cmd;
    cmd.msg_type = MSG_TYPE_COMMAND;
    cmd.seq_num = 1;
    cmd.cmd = CMD_ARM_NOW;
    cmd.reserved = 0;
    cmd.param = 0;
    TEST_ASSERT_EQUAL_UINT8(MSG_TYPE_COMMAND, cmd.msg_type);
    TEST_ASSERT_EQUAL_UINT8(CMD_ARM_NOW, cmd.cmd);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_battery_voltage_to_percent);
    RUN_TEST(test_cap_cell_grid_calculation);
    RUN_TEST(test_distance_haversine);
    RUN_TEST(test_json_telemetry_schema);
    RUN_TEST(test_binary_protocol_structures);
    return UNITY_END();
}
