#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "freertos/semphr.h"
#include "config.h"
#include "feetech.h"

// ---------------------------------------------------------------------------
// Monitor — background FreeRTOS task that runs every TELEMETRY_MS and:
//
//  1. Reads state (position, speed, load, voltage, temperature) from every
//     servo in the joint table.
//  2. Broadcasts a telemetry JSON frame to all connected WebSocket clients.
//  3. Enforces safety thresholds (temperature, load, voltage).
//  4. Implements a command watchdog: if no command is received for
//     WATCHDOG_MS, torque is cut on all servos.
//  5. Writes all events to an SD card log file (blackbox) if a card is
//     present at boot. Logging continues regardless of WiFi state.
//
// Telemetry frame (broadcast to all clients):
//   {
//     "type": "telemetry",
//     "ts":   <millis>,
//     "servos": [
//       { "id":1, "pos":2048, "angle":0.0, "spd":0, "load":12,
//         "volt":74, "temp":35 },
//       ...
//     ]
//   }
//
// Alert frame (on threshold breach):
//   { "type":"alert", "id":1, "reason":"temp", "value":72 }
//   { "type":"alert", "id":0, "reason":"watchdog" }
//   { "type":"alert", "id":1, "reason":"volt",  "value":58 }
// ---------------------------------------------------------------------------

// Returns the most recent joint angles (degrees) read by the telemetry task,
// indexed to match joints[] order (same indexing as fk_compute's angles_deg[]).
// Initialised to default_deg values at boot; updated every TELEMETRY_MS.
// Not mutex-protected — a stale read by at most one telemetry cycle is acceptable
// for FK safety checks since joint limits already bound individual axes.
void monitor_get_angles(float out[SERVO_COUNT]);

// Initialise the monitor. Call once from setup() after feetech_init().
//   ws_handle   — pointer to the AsyncWebSocket instance for broadcasting.
//   bus_mutex   — FreeRTOS mutex that guards all feetech_* calls.
void monitor_start(AsyncWebSocket *ws_handle, SemaphoreHandle_t bus_mutex);

// Call this on every successfully parsed WebSocket command to reset the
// watchdog timer.
void monitor_record_cmd();

// Returns true if the SD card was found and the log file is open.
bool monitor_sd_ok();
