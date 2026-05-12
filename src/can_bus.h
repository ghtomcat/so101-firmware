#pragma once
#include "config.h"

// ---------------------------------------------------------------------------
// CAN bus layer — Airbus-style distributed safety and multi-node voting
//
// This module is gated entirely on CAN_ENABLE in config.h. Setting
// CAN_ENABLE to 0 compiles every function to an empty inline, so callers
// need no #if guards and single-node builds produce identical behaviour.
//
// Hardware: T-CAN485 onboard TJA1050 transceiver.
//   CAN_TX_PIN (26) → TJA1050 TXD
//   CAN_RX_PIN (27) → TJA1050 RXD
//   CAN_SE_PIN (23) → TJA1050 standby (driven LOW to activate)
//
// Message ID scheme (11-bit standard frames):
//
//   Node-specific (low nibble = node_id 1–15):
//     0x100 + node_id  HEARTBEAT   — broadcast every CAN_HEARTBEAT_MS
//     0x110 + node_id  TELEMETRY   — one frame per servo per cycle
//     0x120 + node_id  ALERT       — on threshold breach / watchdog
//     0x130 + node_id  VOTE_REQ    — request quorum before moving
//     0x140 + node_id  VOTE_ACK    — respond to a peer's VOTE_REQ
//
//   Broadcast commands (any node receives):
//     0x200            CMD_MOVE    — move one servo on target node(s)
//     0x201            CMD_STOP    — emergency stop on target node(s)
//
// Voting flow (Airbus N-of-M):
//   1. Initiating node broadcasts VOTE_REQ with servo id + target position.
//   2. Each peer evaluates the move against its own safety state and replies
//      with VOTE_ACK (approve / reject + reason).
//   3. Initiating node waits up to CAN_VOTE_TIMEOUT_MS for CAN_VOTE_QUORUM
//      approvals. On quorum → execute. On timeout or rejection → abort.
//   4. With CAN_VOTE_QUORUM = 1 the node approves its own requests
//      immediately, effectively disabling voting for single-node setups.
// ---------------------------------------------------------------------------

#if CAN_ENABLE
#include <Arduino.h>
#include "joints.h"

// Alert reason codes (shared with monitor.cpp for consistency)
enum CanAlertReason : uint8_t {
    CAN_ALERT_TEMP     = 0,
    CAN_ALERT_LOAD     = 1,
    CAN_ALERT_VOLT     = 2,
    CAN_ALERT_WATCHDOG = 3,
    CAN_ALERT_ESTOP    = 4,
};

// Vote rejection reason codes
enum CanVoteReject : uint8_t {
    REJECT_OUT_OF_RANGE  = 0,
    REJECT_ALERT_ACTIVE  = 1,
    REJECT_TORQUE_OFF    = 2,
};

// Initialise TWAI driver and start the CAN RX task.
// Must be called after feetech_init() and before monitor_start().
void can_init();

// Broadcast this node's heartbeat (called by the monitor task).
// flags: bit0=torque_ok, bit1=calibrated, bit2=sd_ok, bit3=wifi_ok, bit4=alert_active
void can_heartbeat(uint8_t flags);

// Broadcast one servo's telemetry frame (called by the monitor task).
void can_telemetry(uint8_t servo_id, uint16_t step, int16_t load, uint8_t temp, uint8_t volt);

// Broadcast an alert event (called by monitor.cpp on threshold breach).
void can_alert(uint8_t servo_id, CanAlertReason reason, int16_t value);

// Broadcast a CMD_STOP to all nodes (called on local emergency stop so peers
// are notified and can cut their own torque).
void can_stop_all();

// Request a quorum vote before executing a move.
// Returns true if CAN_VOTE_QUORUM approvals are received within
// CAN_VOTE_TIMEOUT_MS, or if CAN_VOTE_QUORUM == 1 (self-approve).
// step is the raw target servo position (already clamped to limits).
bool can_vote_move(uint8_t servo_id, uint16_t step, uint16_t speed);

// Register a callback invoked when a CMD_MOVE arrives over CAN targeting
// this node. The callback runs from the RX task — keep it short.
typedef void (*CanMoveCallback)(uint8_t servo_id, uint16_t step, uint16_t speed, uint8_t acc);
void can_set_move_callback(CanMoveCallback cb);

// Register a callback invoked when a CMD_STOP arrives targeting this node.
typedef void (*CanStopCallback)();
void can_set_stop_callback(CanStopCallback cb);

#else // CAN_ENABLE == 0 — empty stubs, compile to nothing

#include <stdint.h>
#include <stdbool.h>

enum CanAlertReason : uint8_t { CAN_ALERT_TEMP=0, CAN_ALERT_LOAD=1,
                                 CAN_ALERT_VOLT=2, CAN_ALERT_WATCHDOG=3,
                                 CAN_ALERT_ESTOP=4 };

inline void can_init() {}
inline void can_heartbeat(uint8_t) {}
inline void can_telemetry(uint8_t, uint16_t, int16_t, uint8_t, uint8_t) {}
inline void can_alert(uint8_t, CanAlertReason, int16_t) {}
inline void can_stop_all() {}
inline bool can_vote_move(uint8_t, uint16_t, uint16_t) { return true; }

typedef void (*CanMoveCallback)(uint8_t, uint16_t, uint16_t, uint8_t);
typedef void (*CanStopCallback)();
inline void can_set_move_callback(CanMoveCallback) {}
inline void can_set_stop_callback(CanStopCallback) {}

#endif // CAN_ENABLE
