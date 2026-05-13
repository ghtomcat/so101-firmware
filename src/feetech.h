#pragma once
#include <Arduino.h>

// Feetech STS3215 serial servo driver (half-duplex UART)
//
// Packet format:  FF FF ID LEN INSTR [PARAMS...] CHK
// Response:       FF FF ID LEN ERR   [PARAMS...] CHK
// Checksum = ~(ID + LEN + INSTR + params) & 0xFF

// Instructions
#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_READ  0x82
#define ID_BROADCAST    0xFE

// RAM register map (STS3215)
#define REG_TORQUE_EN       40   // 1 byte  (1 = torque on, 0 = off)
#define REG_GOAL_ACC        41   // 1 byte
#define REG_GOAL_POS        42   // 2 bytes (L then H)  — immediately follows Acc
#define REG_GOAL_TIME       44   // 2 bytes (0 = use speed mode)
#define REG_GOAL_SPEED      46   // 2 bytes
#define REG_PRESENT_POS     56   // 2 bytes
#define REG_PRESENT_SPEED   58   // 2 bytes  (bit15 = direction)
#define REG_PRESENT_LOAD    60   // 2 bytes  (bit15 = direction)
#define REG_PRESENT_VOLT    62   // 1 byte   (unit: 0.1V)
#define REG_PRESENT_TEMP    63   // 1 byte   (unit: °C)

struct ServoState {
    uint16_t position;   // 0–4095
    int16_t  speed;      // raw, bit15 = direction
    int16_t  load;       // raw, bit15 = direction
    uint8_t  voltage;    // tenths of volts (e.g. 74 = 7.4V)
    uint8_t  temperature;// °C
};

// Must call once before any other feetech_* functions.
void feetech_init(HardwareSerial &serial, int rx, int tx, int dir, long baud);

bool feetech_ping(uint8_t id);

// Debug ping: sends a PING and captures all raw RX bytes within a 30 ms window.
// Logs sent/received bytes to Serial. buf must be >= 16 bytes.
// Returns number of bytes received (0 = nothing came back).
int feetech_debug_ping(uint8_t id, uint8_t *buf, int buf_len);

// Move servo id to position [0-4095], speed [0-3400 steps/s], acc [0-254]
// Time is set to 0 (speed-controlled mode). Returns true on success.
bool feetech_write_pos(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc);

// Read position, speed, load, voltage, temperature in one burst.
bool feetech_read_state(uint8_t id, ServoState &out);

// Move up to SERVO_COUNT servos simultaneously. All arrays must be count-long.
bool feetech_sync_write_pos(const uint8_t *ids, const uint16_t *pos,
                             const uint16_t *speed, const uint8_t *acc,
                             uint8_t count);

// Read present positions for `count` servos. pos[] receives raw values.
bool feetech_sync_read_pos(const uint8_t *ids, uint16_t *pos, uint8_t count);

// Enable or disable torque on a single servo with verified WRITE (expects ACK).
bool feetech_write_torque(uint8_t id, bool enable);

// Enable or disable torque on a set of servos via a single SYNC_WRITE.
// Pass enable=false for an immediate, bus-level emergency stop.
bool feetech_torque_all(const uint8_t *ids, uint8_t count, bool enable);
