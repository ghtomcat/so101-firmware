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

// Convenience: return only the TCP world position.
Vec3 fk_tcp(const float angles_deg[SERVO_COUNT]);

// Damped-least-squares inverse kinematics (position only, 3-DOF target).
// current_deg: starting joint angles (degrees)
// target:      desired TCP world position (meters)
// result_deg:  output joint angles (degrees) — valid even when returning false
// max_iter:    iteration limit (20 is usually enough)
// tol_m:       convergence tolerance in meters (e.g. 0.001 = 1 mm)
// Returns true if the solver converged within tol_m.
bool ik_solve(const float current_deg[SERVO_COUNT], Vec3 target,
              float result_deg[SERVO_COUNT], int max_iter, float tol_m);
