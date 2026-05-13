#include "kinematics.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static const float PI_F    = 3.14159265358979323846f;
static const float HALF_PI = 1.57079632679489661923f;
static const float DEG2RAD = 3.14159265358979323846f / 180.0f;

// ---------------------------------------------------------------------------
// 4×4 homogeneous transform (row-major)
// ---------------------------------------------------------------------------

struct Mat4 { float m[4][4]; };

static inline float dot3(Vec3 a, Vec3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3  sub3(Vec3 a, Vec3 b)    { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline Vec3  add3(Vec3 a, Vec3 b)    { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vec3  sc3 (Vec3 v, float s)   { return {v.x*s,   v.y*s,   v.z*s  }; }
static inline float lsq3(Vec3 v)            { return dot3(v, v); }
static inline float cl01(float v)           { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

static Mat4 eye() {
    Mat4 r = {};
    r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
    return r;
}

static Mat4 mmul(const Mat4 &a, const Mat4 &b) {
    Mat4 r = {};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

static Vec3 mpos(const Mat4 &m) { return {m.m[0][3], m.m[1][3], m.m[2][3]}; }

static Mat4 transl(float x, float y, float z) {
    Mat4 r = eye();
    r.m[0][3] = x; r.m[1][3] = y; r.m[2][3] = z;
    return r;
}
static Mat4 rx(float a) {
    Mat4 r = eye(); float c = cosf(a), s = sinf(a);
    r.m[1][1]=c; r.m[1][2]=-s; r.m[2][1]=s; r.m[2][2]=c; return r;
}
static Mat4 ry(float a) {
    Mat4 r = eye(); float c = cosf(a), s = sinf(a);
    r.m[0][0]=c; r.m[0][2]=s; r.m[2][0]=-s; r.m[2][2]=c; return r;
}
static Mat4 rz(float a) {
    Mat4 r = eye(); float c = cosf(a), s = sinf(a);
    r.m[0][0]=c; r.m[0][1]=-s; r.m[1][0]=s; r.m[1][1]=c; return r;
}

// URDF RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)
static Mat4 rpy(float ro, float pi, float ya) {
    return mmul(rz(ya), mmul(ry(pi), rx(ro)));
}
static Mat4 xf(float x, float y, float z, float ro, float pi, float ya) {
    return mmul(transl(x, y, z), rpy(ro, pi, ya));
}

// ---------------------------------------------------------------------------
// SO-101 joint origins from so101_new_calib.urdf
// Each row: [tx, ty, tz, roll, pitch, yaw] — URDF <origin xyz="..." rpy="..."/>
// Index 0 is the world base; indices 1-6 are the six revolute joints.
// ---------------------------------------------------------------------------

static const float ORIG[SERVO_COUNT + 1][6] = {
    /* [0] base        */ {  0.0f,          0.0f,         0.0f,      0.0f,    0.0f,      0.0f   },
    /* [1] shoulder_pan*/ {  0.0388353f,    0.0f,         0.0624f,   PI_F,    0.0f,     -PI_F   },
    /* [2] shldr_lift  */ { -0.0303992f,   -0.0182778f,  -0.0542f,  -HALF_PI,-HALF_PI,  0.0f   },
    /* [3] elbow_flex  */ { -0.11257f,     -0.028f,       0.0f,      0.0f,    0.0f,     HALF_PI },
    /* [4] wrist_flex  */ { -0.1349f,       0.0052f,      0.0f,      0.0f,    0.0f,    -HALF_PI },
    /* [5] wrist_roll  */ {  0.0f,         -0.0611f,      0.0181f,   HALF_PI, 0.0486795f,PI_F  },
    /* [6] gripper     */ {  0.0202f,       0.0188f,     -0.0234f,   HALF_PI, 0.0f,      0.0f  },
};

// Gripper-frame fixed offset gives the tool-centre-point (gripper_frame_joint, fixed).
static const float TCP[6] = { -0.0079f, -0.000218f, -0.0981274f, 0.0f, PI_F, 0.0f };

// ---------------------------------------------------------------------------
// Conservative capsule radii per link (meters).  Link i: nodes[i]→nodes[i+1].
// Values sized to enclose SO-101 link cross-sections at full servo housing.
// ---------------------------------------------------------------------------

static const float RAD[7] = {
    0.060f,  // link 0: base + pan housing
    0.042f,  // link 1: shoulder section
    0.040f,  // link 2: upper arm
    0.038f,  // link 3: forearm
    0.032f,  // link 4: wrist module
    0.028f,  // link 5: gripper link
    0.025f,  // link 6: gripper jaw / frame
};

// ---------------------------------------------------------------------------
// Segment–segment squared distance
// Ericson, "Real-Time Collision Detection", §5.1.9
// ---------------------------------------------------------------------------

static float seg_dist_sq(Vec3 p1, Vec3 p2, Vec3 q1, Vec3 q2) {
    Vec3 d1 = sub3(p2,p1), d2 = sub3(q2,q1), r = sub3(p1,q1);
    float a = lsq3(d1), e = lsq3(d2), f = dot3(d2,r);
    float s, t;

    if (a <= 1e-10f) {
        if (e <= 1e-10f) return lsq3(r);
        s = 0.0f; t = cl01(f / e);
    } else {
        float c = dot3(d1,r);
        if (e <= 1e-10f) {
            t = 0.0f; s = cl01(-c / a);
        } else {
            float b = dot3(d1,d2), den = a*e - b*b;
            s = (den > 1e-10f) ? cl01((b*f - c*e) / den) : 0.0f;
            t = (b*s + f) / e;
            if      (t < 0.0f) { t = 0.0f; s = cl01(-c / a); }
            else if (t > 1.0f) { t = 1.0f; s = cl01((b-c) / a); }
        }
    }
    return lsq3(sub3(add3(p1, sc3(d1,s)), add3(q1, sc3(d2,t))));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void fk_compute(const float angles_deg[SERVO_COUNT], FkResult *out) {
    Mat4 T = eye();
    out->nodes[0] = mpos(T);   // base at world origin

    for (int i = 0; i < SERVO_COUNT; i++) {
        const float *o = ORIG[i + 1];
        T = mmul(T, xf(o[0],o[1],o[2], o[3],o[4],o[5]));
        out->nodes[i + 1] = mpos(T);
        T = mmul(T, rz(angles_deg[i] * DEG2RAD));
    }

    // TCP: fixed offset from last joint frame
    T = mmul(T, xf(TCP[0],TCP[1],TCP[2], TCP[3],TCP[4],TCP[5]));
    out->nodes[7] = mpos(T);
}

bool fk_check(const float angles_deg[SERVO_COUNT], char *reason_out) {
    FkResult fk;
    fk_compute(angles_deg, &fk);

    // Ground clearance: no node below the mounting plane.
    // FK_MIN_HEIGHT_M in config.h (default 0.0 = table surface).
    for (int i = 1; i < FK_NODES; i++) {
        if (fk.nodes[i].z < FK_MIN_HEIGHT_M) {
            if (reason_out) snprintf(reason_out, 32, "below_floor:%d", i);
            return false;
        }
    }

    // Maximum reach: TCP distance from base origin.
    if (lsq3(fk.nodes[7]) > FK_MAX_REACH_M * FK_MAX_REACH_M) {
        if (reason_out) snprintf(reason_out, 32, "over_reach");
        return false;
    }

    // Self-collision: all non-adjacent link pairs (|i-j| >= 2).
    static const int8_t PAIRS[][2] = {
        {0,2},{0,3},{0,4},{0,5},{0,6},
        {1,3},{1,4},{1,5},{1,6},
        {2,4},{2,5},{2,6},
        {3,5},{3,6},
        {4,6},
    };
    static const int N = sizeof(PAIRS) / sizeof(PAIRS[0]);

    for (int p = 0; p < N; p++) {
        int a = PAIRS[p][0], b = PAIRS[p][1];
        float r = RAD[a] + RAD[b];
        if (seg_dist_sq(fk.nodes[a], fk.nodes[a+1],
                        fk.nodes[b], fk.nodes[b+1]) < r * r) {
            if (reason_out) snprintf(reason_out, 32, "collision:%d-%d", a, b);
            return false;
        }
    }

    return true;
}
