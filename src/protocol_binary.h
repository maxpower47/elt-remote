#ifndef PROTOCOL_BINARY_H
#define PROTOCOL_BINARY_H

#include <stdint.h>
#include <string.h>

#pragma pack(push, 1)

// Message Type Identifiers
#define MSG_TYPE_TELEMETRY   0x01
#define MSG_TYPE_COMMAND     0x02

// Command Identifiers
#define CMD_DISARM           0x01
#define CMD_ARM_NOW          0x02
#define CMD_ARM_TIMER        0x03

// State Identifiers
#define STATE_ID_DISARMED    0x00
#define STATE_ID_ARMED_TIMER 0x01
#define STATE_ID_ACTIVE      0x02

// Telemetry Flags
#define FLAG_USB_POWER       0x01
#define FLAG_GPS_VALID       0x02

// Compact 20-Byte Over-the-Air Telemetry Packet
typedef struct {
    uint8_t  msg_type;      // 0x01 (MSG_TYPE_TELEMETRY)
    uint8_t  seq_num;       // Rolling sequence number
    uint8_t  state;         // STATE_ID_*
    uint8_t  flags;         // FLAG_* bitmask
    uint16_t batt_mv;       // Battery millivolts (e.g. 4120 = 4.12V)
    uint32_t remaining_sec; // Countdown seconds for ARMED_TIMER
    int32_t  lat_e7;        // Latitude * 1e7
    int32_t  lon_e7;        // Longitude * 1e7
} LoRaTelemetryPacket;

// Compact 8-Byte Over-the-Air Control Command Packet
typedef struct {
    uint8_t  msg_type;      // 0x02 (MSG_TYPE_COMMAND)
    uint8_t  seq_num;       // Rolling sequence number
    uint8_t  cmd;           // CMD_DISARM, CMD_ARM_NOW, CMD_ARM_TIMER
    uint8_t  reserved;      // 0x00 padding
    uint32_t param;         // Countdown seconds (if CMD_ARM_TIMER)
} LoRaCommandPacket;

#pragma pack(pop)

#endif // PROTOCOL_BINARY_H
