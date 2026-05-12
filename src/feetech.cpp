#include "feetech.h"

static HardwareSerial *_serial = nullptr;
static int _dir_pin = -1;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static uint8_t checksum(const uint8_t *buf, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    return ~sum;
}

static void tx_mode() {
    digitalWrite(_dir_pin, HIGH);
    delayMicroseconds(2);
}

static void rx_mode() {
    // Wait for last byte to fully clock out at 1 Mbps (10 bits = 10 µs), add margin.
    _serial->flush();
    delayMicroseconds(20);
    digitalWrite(_dir_pin, LOW);
}

static void send_packet(uint8_t id, uint8_t instr,
                        const uint8_t *params, uint8_t param_len) {
    uint8_t len = param_len + 2; // covers instruction + params + checksum byte
    uint8_t s   = id + len + instr;
    for (int i = 0; i < param_len; i++) s += params[i];
    uint8_t chk = ~s;

    tx_mode();
    _serial->write(0xFF);
    _serial->write(0xFF);
    _serial->write(id);
    _serial->write(len);
    _serial->write(instr);
    if (param_len) _serial->write(params, param_len);
    _serial->write(chk);
    rx_mode();
}

// Returns number of data bytes in response, -1 on timeout/error.
// out_data must be at least 16 bytes.
static int recv_packet(uint8_t expected_id, uint8_t *out_data, int timeout_ms = 10) {
    uint32_t deadline = millis() + timeout_ms;

    // Sync to header (FF FF)
    int hdr = 0;
    while (millis() < deadline) {
        if (_serial->available()) {
            uint8_t b = _serial->read();
            if (b == 0xFF) { hdr++; if (hdr >= 2) break; }
            else hdr = 0;
        }
    }
    if (hdr < 2) return -1;

    // Read ID, LEN
    while (millis() < deadline && _serial->available() < 2) {}
    if (_serial->available() < 2) return -1;
    uint8_t id  = _serial->read();
    uint8_t len = _serial->read(); // = data_bytes + 2

    if (id != expected_id || len < 2) return -1;

    int data_len = len - 2; // error byte + data bytes - checksum byte
    // len = ERR + data + CHK  =>  data count = len - 2
    int total = len; // ERR + data_bytes + CHK

    while (millis() < deadline && _serial->available() < total) {}
    if (_serial->available() < total) return -1;

    uint8_t err = _serial->read();
    for (int i = 0; i < data_len; i++) out_data[i] = _serial->read();
    uint8_t chk_recv = _serial->read();

    // Verify checksum
    uint8_t s = id + len + err;
    for (int i = 0; i < data_len; i++) s += out_data[i];
    if ((uint8_t)~s != chk_recv) return -1;
    if (err) return -1;

    return data_len;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void feetech_init(HardwareSerial &serial, int rx, int tx, int dir, long baud) {
    _serial  = &serial;
    _dir_pin = dir;
    pinMode(dir, OUTPUT);
    digitalWrite(dir, LOW); // start in RX mode
    serial.begin(baud, SERIAL_8N1, rx, tx);
    delay(100);
}

bool feetech_ping(uint8_t id) {
    _serial->flush();
    while (_serial->available()) _serial->read(); // drain

    send_packet(id, INST_PING, nullptr, 0);

    uint8_t buf[4];
    return recv_packet(id, buf) >= 0;
}

bool feetech_write_pos(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc) {
    // Write 8 bytes starting at REG_GOAL_ACC (41):
    // [acc, 0, pos_l, pos_h, time_l=0, time_h=0, speed_l, speed_h]
    uint8_t params[10];
    params[0] = REG_GOAL_ACC;     // start address
    params[1] = acc;
    params[2] = 0;                // padding register 42
    params[3] = pos & 0xFF;
    params[4] = (pos >> 8) & 0xFF;
    params[5] = 0;                // time = 0 (speed mode)
    params[6] = 0;
    params[7] = speed & 0xFF;
    params[8] = (speed >> 8) & 0xFF;

    // WRITE_DATA: params = [start_addr, data...]
    send_packet(id, INST_WRITE, params, 9);

    uint8_t buf[4];
    return recv_packet(id, buf) >= 0;
}

bool feetech_read_state(uint8_t id, ServoState &out) {
    // Read 8 bytes from REG_PRESENT_POS (56): pos(2) speed(2) load(2) volt(1) temp(1)
    uint8_t params[2] = {REG_PRESENT_POS, 8};
    _serial->flush();
    while (_serial->available()) _serial->read();

    send_packet(id, INST_READ, params, 2);

    uint8_t buf[8];
    int n = recv_packet(id, buf, 15);
    if (n < 8) return false;

    out.position    = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    out.speed       = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    out.load        = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    out.voltage     = buf[6];
    out.temperature = buf[7];
    return true;
}

bool feetech_sync_write_pos(const uint8_t *ids, const uint16_t *pos,
                             const uint16_t *speed, const uint8_t *acc,
                             uint8_t count) {
    // SYNC_WRITE: FF FF FE LEN 83 START DATA_LEN [ID DATA...]*n CHK
    // DATA_LEN = 8 bytes per servo (see feetech_write_pos layout)
    const uint8_t data_per_servo = 8;
    // params = [start_addr, data_len, ID0, d0..d7, ID1, d0..d7, ...]
    int param_len = 2 + count * (1 + data_per_servo);
    uint8_t params[2 + 6 * 9]; // max 6 servos

    params[0] = REG_GOAL_ACC;
    params[1] = data_per_servo;

    for (int i = 0; i < count; i++) {
        uint8_t *p = &params[2 + i * 9];
        p[0] = ids[i];
        p[1] = acc[i];
        p[2] = 0;
        p[3] = pos[i] & 0xFF;
        p[4] = (pos[i] >> 8) & 0xFF;
        p[5] = 0;
        p[6] = 0;
        p[7] = speed[i] & 0xFF;
        p[8] = (speed[i] >> 8) & 0xFF;
    }

    // SYNC_WRITE goes to broadcast ID (0xFE), no response expected.
    send_packet(ID_BROADCAST, INST_SYNC_WRITE, params, param_len);
    return true;
}

bool feetech_sync_read_pos(const uint8_t *ids, uint16_t *pos, uint8_t count) {
    // SYNC_READ: each servo replies with a status packet in order.
    uint8_t params[2 + 6]; // start_addr, data_len, id0..idN
    params[0] = REG_PRESENT_POS;
    params[1] = 2; // 2 bytes (position only)
    for (int i = 0; i < count; i++) params[2 + i] = ids[i];

    _serial->flush();
    while (_serial->available()) _serial->read();

    send_packet(ID_BROADCAST, INST_SYNC_READ, params, 2 + count);

    for (int i = 0; i < count; i++) {
        uint8_t buf[4];
        int n = recv_packet(ids[i], buf, 15);
        if (n < 2) return false;
        pos[i] = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return true;
}

bool feetech_torque_all(const uint8_t *ids, uint8_t count, bool enable) {
    // SYNC_WRITE torque enable register (0x28) for all servos in one packet.
    // params = [start_addr, data_len_per_servo, ID0, val0, ID1, val1, ...]
    uint8_t params[2 + 6 * 2]; // max 6 servos × (id + 1 data byte)
    params[0] = REG_TORQUE_EN;
    params[1] = 1; // 1 byte of data per servo

    for (int i = 0; i < count; i++) {
        params[2 + i * 2]     = ids[i];
        params[2 + i * 2 + 1] = enable ? 1 : 0;
    }

    send_packet(ID_BROADCAST, INST_SYNC_WRITE, params, 2 + count * 2);
    return true;
}
