#include "can_bus.h"

#if CAN_ENABLE

#include <Arduino.h>
#include "driver/twai.h"
#include "joints.h"
#include "feetech.h"
#include "freertos/semphr.h"

// ---------------------------------------------------------------------------
// Private state
// ---------------------------------------------------------------------------

static CanMoveCallback _move_cb = nullptr;
static CanStopCallback _stop_cb = nullptr;

// Pending vote state — only one vote in flight at a time.
static struct {
    volatile bool    active;
    volatile uint8_t seq;
    volatile uint8_t approvals;
    volatile uint8_t rejections;
    SemaphoreHandle_t done;
} _vote;

static volatile bool _alert_active = false; // set by monitor via can_alert()

// ---------------------------------------------------------------------------
// CAN message ID helpers
// ---------------------------------------------------------------------------

#define MSG_HEARTBEAT(n)  (0x100u + (n))
#define MSG_TELEMETRY(n)  (0x110u + (n))
#define MSG_ALERT(n)      (0x120u + (n))
#define MSG_VOTE_REQ(n)   (0x130u + (n))
#define MSG_VOTE_ACK(n)   (0x140u + (n))
#define MSG_CMD_MOVE      0x200u
#define MSG_CMD_STOP      0x201u

// ---------------------------------------------------------------------------
// Low-level transmit
// ---------------------------------------------------------------------------

static void twai_send(uint32_t id, const uint8_t *data, uint8_t dlc) {
    twai_message_t msg{};
    msg.identifier         = id;
    msg.data_length_code   = dlc;
    msg.extd               = 0; // standard 11-bit frame
    memcpy(msg.data, data, dlc);
    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

// ---------------------------------------------------------------------------
// RX task — handles incoming CAN frames
// ---------------------------------------------------------------------------

static void rx_task(void *) {
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) continue;

        uint32_t id = msg.identifier;

        // --- CMD_MOVE ---
        if (id == MSG_CMD_MOVE && msg.data_length_code >= 7) {
            uint8_t  target = msg.data[0];
            uint8_t  sid    = msg.data[1];
            uint16_t step   = (uint16_t)msg.data[2] | ((uint16_t)msg.data[3] << 8);
            uint16_t speed  = (uint16_t)msg.data[4] | ((uint16_t)msg.data[5] << 8);
            uint8_t  acc    = msg.data[6];

            if ((target == CAN_NODE_ID || target == 0xFF) && _move_cb) {
                _move_cb(sid, step, speed, acc);
            }
            continue;
        }

        // --- CMD_STOP ---
        if (id == MSG_CMD_STOP) {
            uint8_t target = msg.data_length_code > 0 ? msg.data[0] : 0xFF;
            if ((target == CAN_NODE_ID || target == 0xFF) && _stop_cb) {
                _stop_cb();
            }
            continue;
        }

        // --- VOTE_REQ from another node ---
        // A peer is asking us to approve a planned move.
        uint32_t base = id & 0x1F0u;
        if (base == 0x130u && msg.data_length_code >= 7) {
            uint8_t  requesting_node = msg.data[0];
            uint8_t  seq             = msg.data[1];
            uint8_t  sid             = msg.data[2];
            uint16_t target_step     = (uint16_t)msg.data[3] | ((uint16_t)msg.data[4] << 8);
            // speed in data[5-6] (available if needed for future checks)

            uint8_t vote   = 1; // approve
            uint8_t reason = 0;

            // Reject if we're in an alert state — the network isn't healthy
            if (_alert_active) {
                vote = 0; reason = REJECT_ALERT_ACTIVE;
            } else {
                // Check that the proposed step is within this node's limits for that servo
                const JointConfig *j = joint_by_id(sid);
                if (!j) {
                    vote = 0; reason = REJECT_OUT_OF_RANGE;
                } else {
                    float deg = 0.0f;
                    step_to_angle(sid, target_step, &deg);
                    if (deg < j->min_deg || deg > j->max_deg) {
                        vote = 0; reason = REJECT_OUT_OF_RANGE;
                    }
                }
            }

            // Send VOTE_ACK back to the requesting node
            uint8_t ack[5] = {
                CAN_NODE_ID,
                requesting_node,
                seq,
                vote,
                reason
            };
            twai_send(MSG_VOTE_ACK(CAN_NODE_ID), ack, 5);
            continue;
        }

        // --- VOTE_ACK addressed to us ---
        if (base == 0x140u && msg.data_length_code >= 4) {
            uint8_t responding_to = msg.data[1];
            uint8_t seq           = msg.data[2];
            uint8_t result        = msg.data[3];

            if (responding_to == CAN_NODE_ID && _vote.active && seq == _vote.seq) {
                if (result == 1) {
                    _vote.approvals++;
                } else {
                    _vote.rejections++;
                }
                // Wake the waiting thread as soon as quorum is reached
                if (_vote.approvals >= CAN_VOTE_QUORUM) {
                    xSemaphoreGive(_vote.done);
                }
            }
            continue;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void can_init() {
    // Activate the TJA1050 transceiver (standby pin LOW = active)
    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);

    // Select timing config based on CAN_BAUD_KBPS
#if   CAN_BAUD_KBPS == 1000
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BAUD_KBPS == 500
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BAUD_KBPS == 250
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
#elif CAN_BAUD_KBPS == 125
    twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
#else
    #error "Unsupported CAN_BAUD_KBPS — choose 125, 250, 500, or 1000"
#endif

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK ||
        twai_start()                   != ESP_OK) {
        Serial.println("[can]  driver init failed");
        return;
    }

    _vote.done = xSemaphoreCreateBinary();

    xTaskCreate(rx_task, "can_rx", 3072, nullptr, 2, nullptr);
    Serial.printf("[can]  node_id=%u baud=%u kbps quorum=%u\n",
                  CAN_NODE_ID, CAN_BAUD_KBPS, CAN_VOTE_QUORUM);
}

void can_heartbeat(uint8_t flags) {
    uint32_t up = millis() / 1000;
    uint8_t  d[8] = {
        CAN_NODE_ID,
        flags,
        (uint8_t)(up & 0xFF),
        (uint8_t)(up >> 8),
        0, 0, 0, 0
    };
    twai_send(MSG_HEARTBEAT(CAN_NODE_ID), d, 8);
}

void can_telemetry(uint8_t servo_id, uint16_t step, int16_t load,
                   uint8_t temp, uint8_t volt) {
    uint8_t d[8] = {
        CAN_NODE_ID,
        servo_id,
        (uint8_t)(step & 0xFF), (uint8_t)(step >> 8),
        (uint8_t)((uint16_t)load & 0xFF), (uint8_t)((uint16_t)load >> 8),
        temp,
        volt
    };
    twai_send(MSG_TELEMETRY(CAN_NODE_ID), d, 8);
}

void can_alert(uint8_t servo_id, CanAlertReason reason, int16_t value) {
    _alert_active = true;
    uint8_t d[8] = {
        CAN_NODE_ID,
        servo_id,
        (uint8_t)reason,
        (uint8_t)((uint16_t)value & 0xFF), (uint8_t)((uint16_t)value >> 8),
        0, 0, 0
    };
    twai_send(MSG_ALERT(CAN_NODE_ID), d, 8);
}

void can_stop_all() {
    uint8_t d[2] = {0xFF, 0}; // target = broadcast
    twai_send(MSG_CMD_STOP, d, 2);
}

bool can_vote_move(uint8_t servo_id, uint16_t step, uint16_t speed) {
    // With quorum of 1 this node self-approves without any bus traffic.
    if (CAN_VOTE_QUORUM <= 1) return true;

    static uint8_t seq = 0;
    seq++;

    _vote.active     = true;
    _vote.seq        = seq;
    _vote.approvals  = 0;
    _vote.rejections = 0;

    // Broadcast vote request
    uint8_t d[7] = {
        CAN_NODE_ID,
        seq,
        servo_id,
        (uint8_t)(step & 0xFF), (uint8_t)(step >> 8),
        (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)
    };
    twai_send(MSG_VOTE_REQ(CAN_NODE_ID), d, 7);

    // Wait for quorum (the RX task gives the semaphore when it arrives)
    bool ok = xSemaphoreTake(_vote.done, pdMS_TO_TICKS(CAN_VOTE_TIMEOUT_MS)) == pdTRUE;
    _vote.active = false;

    if (!ok) {
        Serial.printf("[can]  vote timeout seq=%u servo=%u\n", seq, servo_id);
    }
    return ok && (_vote.approvals >= CAN_VOTE_QUORUM);
}

void can_set_move_callback(CanMoveCallback cb) { _move_cb = cb; }
void can_set_stop_callback(CanStopCallback cb) { _stop_cb = cb; }

#endif // CAN_ENABLE
