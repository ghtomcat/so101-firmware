#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// 3-vector (meters, world frame)
typedef struct { float x, y, z; } Vec3;

// FK result: world-frame positions of every node in the kinematic chain.
//   nodes[0]     — base origin (always 0,0,0)
//   nodes[1..6]  — joint frame origins (shoulder_pan … gripper)
//   nodes[7]     — tool-centre-point (gripper_frame_link / tip)
#define FK_NODES 8

typedef struct {
    Vec3 nodes[FK_NODES];
} FkResult;

// Compute forward kinematics for the given joint angles (degrees).
// Angles must be indexed to match joints[] order in joints.h:
//   [0]=shoulder_pan  [1]=shoulder_lift  [2]=elbow_flex
//   [3]=wrist_flex    [4]=wrist_roll      [5]=gripper
void fk_compute(const float angles_deg[SERVO_COUNT], FkResult *out);

// Returns true when the pose is collision-free and within the safety envelope.
// On false, writes a short rejection tag (max 31 chars + NUL) to reason_out.
// reason_out may be NULL.
bool fk_check(const float angles_deg[SERVO_COUNT], char *reason_out);
