#include "joints.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Default SO-101 joint table
//
// These values are safe starting defaults. After assembly, run the calibration
// procedure (see docs/api.md) to set the correct center_step and direction
// for each joint based on your specific servo mounting.
//
// Joint limits match the SO-101 mechanical design. Tighten min_deg / max_deg
// if your specific build has a more restricted range of motion.
// ---------------------------------------------------------------------------

JointConfig joints[SERVO_COUNT] = {
    // id  center  dir  min     max    default  name
    {  1,  2048,  +1,  -180.0f, +180.0f,  0.0f, "base"        },
    {  2,  2048,  +1,   -90.0f,  +90.0f,  0.0f, "shoulder"    },
    {  3,  2048,  +1,   -90.0f,  +90.0f, +90.0f,"elbow"       },
    {  4,  2048,  +1,   -90.0f,  +90.0f,  0.0f, "wrist_pitch" },
    {  5,  2048,  +1,  -180.0f, +180.0f,  0.0f, "wrist_roll"  },
    {  6,  2048,  +1,    0.0f,   +90.0f,  0.0f, "gripper"     },
};

// ---------------------------------------------------------------------------

const JointConfig *joint_by_id(uint8_t id) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (joints[i].id == id) return &joints[i];
    }
    return nullptr;
}

bool angle_to_step(uint8_t id, float deg, uint16_t *step_out) {
    const JointConfig *j = joint_by_id(id);
    if (!j) return false;

    // Clamp to joint limits
    if (deg < j->min_deg) deg = j->min_deg;
    if (deg > j->max_deg) deg = j->max_deg;

    float raw = (float)j->center_step + j->direction * deg * STEPS_PER_DEG;

    // Clamp to valid servo step range
    if (raw < 0.0f)      raw = 0.0f;
    if (raw > 4095.0f)   raw = 4095.0f;

    *step_out = (uint16_t)roundf(raw);
    return true;
}

bool step_to_angle(uint8_t id, uint16_t step, float *deg_out) {
    const JointConfig *j = joint_by_id(id);
    if (!j) return false;

    *deg_out = (float)j->direction * ((float)step - (float)j->center_step) / STEPS_PER_DEG;
    return true;
}
