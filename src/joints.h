#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Joint configuration and angle ↔ step conversion for the SO-101 arm
//
// The STS3215 servo has 4096 steps per full revolution (0.088° resolution).
// Each joint maps a physical angle in degrees to a raw step value sent to
// the servo. The mapping is:
//
//   step = center_step + direction * round(deg * STEPS_PER_DEG)
//
// center_step  — raw step that corresponds to 0° for this joint.
//                Set during calibration; defaults to 2048 (mid-range).
// direction    — +1 if positive angle increases step count,
//                -1 if positive angle decreases step count.
//                Depends on how the servo is physically mounted.
// min_deg / max_deg — software joint limits. Angles outside this range are
//                     clamped before being sent to the servo.
// default_deg  — safe home position used by move_home command.
// ---------------------------------------------------------------------------

#define STEPS_PER_DEG  (4096.0f / 360.0f)   // ≈ 11.378 steps per degree

struct JointConfig {
    uint8_t     id;           // Feetech servo ID (1–6)
    uint16_t    center_step;  // raw step value at 0°
    int8_t      direction;    // +1 or -1
    float       min_deg;      // minimum allowed angle (degrees)
    float       max_deg;      // maximum allowed angle (degrees)
    float       default_deg;  // safe home position (degrees)
    const char *name;         // human-readable joint label
};

// Default SO-101 joint table. center_step and direction are set to safe
// defaults and MUST be updated after physical calibration (see docs/api.md).
extern JointConfig joints[SERVO_COUNT];

// Return the JointConfig for a given servo ID, or nullptr if not found.
const JointConfig *joint_by_id(uint8_t id);

// Convert degrees to a raw step value for the given servo ID.
// Clamps deg to [min_deg, max_deg] before conversion.
// Returns false if the servo ID is not in the joint table.
bool angle_to_step(uint8_t id, float deg, uint16_t *step_out);

// Convert a raw step value to degrees for the given servo ID.
// Returns false if the servo ID is not in the joint table.
bool step_to_angle(uint8_t id, uint16_t step, float *deg_out);
