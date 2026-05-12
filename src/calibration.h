#pragma once
#include <Arduino.h>
#include "config.h"
#include "joints.h"

// ---------------------------------------------------------------------------
// Calibration — end-stop sweep and NVS persistence
//
// Calibration finds each joint's physical range of motion by sweeping slowly
// until the load spikes at each end-stop, then computes:
//
//   center_step = (min_step + max_step) / 2
//   range_deg   = (max_step - min_step) / STEPS_PER_DEG
//   min_deg     = -(range_deg / 2) + margin
//   max_deg     = +(range_deg / 2) - margin
//
// Results are written to ESP32 NVS (flash) under the namespace "so101cal"
// and are automatically loaded at boot. The SD blackbox logs calibration
// events but is not the source of truth.
//
// NVS key scheme (max 15 chars):
//   "jN_cen"  uint16  center_step for joint N
//   "jN_dir"  int8    direction    for joint N
//   "jN_min"  float   min_deg      for joint N
//   "jN_max"  float   max_deg      for joint N
// ---------------------------------------------------------------------------

// Load saved calibration from NVS into the joints[] table.
// Call once from setup() before monitor_start().
// Silently skips joints with no saved data (keeps defaults from joints.cpp).
void cal_load();

// Save the current joints[] table to NVS.
// Called automatically after a successful calibrate command.
void cal_save();

// Erase all calibration data from NVS, reverting to firmware defaults.
void cal_erase();

// Result of a single-joint calibration sweep.
struct CalResult {
    bool     ok;
    uint8_t  id;
    uint16_t min_step;
    uint16_t max_step;
    uint16_t center_step;
    float    range_deg;
    float    min_deg;
    float    max_deg;
};

// Perform an end-stop sweep on one joint.
//   id          — servo ID to calibrate
//   speed       — sweep speed in steps/s (recommended: 100–200)
//   margin_deg  — degrees to subtract from each limit as a safety buffer
//   load_thresh — load value that indicates an end-stop has been reached
//   timeout_ms  — max time to reach each end-stop before giving up
//
// The function blocks for up to 2 × timeout_ms. During the sweep the bus
// mutex must NOT be held by the caller — this function takes it internally.
CalResult cal_sweep(uint8_t id, uint16_t speed, float margin_deg,
                    uint16_t load_thresh, uint32_t timeout_ms);
