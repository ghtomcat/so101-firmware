#include "calibration.h"
#include "feetech.h"
#include "monitor.h"
#include <Preferences.h>
#include <math.h>
#include "freertos/semphr.h"

// NVS namespace
static const char *NVS_NS = "so101cal";

// The bus mutex is owned by main.cpp; calibration.cpp receives it via cal_sweep().
// Stored here so NVS helpers don't need it.
extern SemaphoreHandle_t bus_mutex;

// ---------------------------------------------------------------------------
// NVS helpers
// ---------------------------------------------------------------------------

// Build a per-joint NVS key, e.g. "j1_cen" (max 15 chars — NVS limit).
static void make_key(char *buf, uint8_t id, const char *field) {
    snprintf(buf, 16, "j%u_%s", (unsigned)id, field);
}

void cal_load() {
    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/true);

    for (int i = 0; i < SERVO_COUNT; i++) {
        uint8_t id = joints[i].id;
        char key[16];

        make_key(key, id, "cen");
        if (prefs.isKey(key)) {
            joints[i].center_step = (uint16_t)prefs.getUInt(key, joints[i].center_step);
        }

        make_key(key, id, "dir");
        if (prefs.isKey(key)) {
            joints[i].direction = (int8_t)prefs.getInt(key, joints[i].direction);
        }

        make_key(key, id, "min");
        if (prefs.isKey(key)) {
            joints[i].min_deg = prefs.getFloat(key, joints[i].min_deg);
        }

        make_key(key, id, "max");
        if (prefs.isKey(key)) {
            joints[i].max_deg = prefs.getFloat(key, joints[i].max_deg);
        }
    }

    prefs.end();
    Serial.println("[cal]   calibration loaded from NVS");
}

void cal_save() {
    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/false);

    for (int i = 0; i < SERVO_COUNT; i++) {
        uint8_t id = joints[i].id;
        char key[16];

        make_key(key, id, "cen"); prefs.putUInt  (key, joints[i].center_step);
        make_key(key, id, "dir"); prefs.putInt   (key, joints[i].direction);
        make_key(key, id, "min"); prefs.putFloat (key, joints[i].min_deg);
        make_key(key, id, "max"); prefs.putFloat (key, joints[i].max_deg);
    }

    prefs.end();
    Serial.println("[cal]   calibration saved to NVS");
}

void cal_erase() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.clear();
    prefs.end();
    Serial.println("[cal]   NVS calibration erased — firmware defaults active");
}

// ---------------------------------------------------------------------------
// End-stop sweep
// ---------------------------------------------------------------------------

CalResult cal_sweep(uint8_t id, uint16_t speed, float margin_deg,
                    uint16_t load_thresh, uint32_t timeout_ms) {
    CalResult r{};
    r.id = id;

    const JointConfig *j = joint_by_id(id);
    if (!j) {
        Serial.printf("[cal]   unknown joint id=%u\n", id);
        return r;
    }

    // -----------------------------------------------------------------------
    // Phase 1 — sweep toward max_step (positive direction)
    // -----------------------------------------------------------------------
    Serial.printf("[cal]   id=%u sweeping toward max...\n", id);

    // Enable torque, wait for servo to be ready, then command to max.
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    bool torque_ok = feetech_write_torque(id, true);
    xSemaphoreGive(bus_mutex);
    Serial.printf("[cal]   id=%u torque_enable=%s\n", id, torque_ok ? "OK" : "FAIL");
    if (!torque_ok) {
        // Retry once — first attempt may lose the response if bus was idle.
        delay(20);
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        torque_ok = feetech_write_torque(id, true);
        xSemaphoreGive(bus_mutex);
        Serial.printf("[cal]   id=%u torque_enable retry=%s\n", id, torque_ok ? "OK" : "FAIL");
    }

    delay(50); // let servo engage torque before commanding position

    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    bool pos_ok = feetech_write_pos(id, 4095, speed, 10);
    xSemaphoreGive(bus_mutex);
    Serial.printf("[cal]   id=%u pos_cmd 4095 speed=%u: %s\n", id, speed, pos_ok ? "OK" : "FAIL");

    // End-stop detection: servo must first travel MIN_TRAVEL steps away from
    // its starting position (proving it is actually moving), then be stable
    // for STABLE_COUNT × 50 ms (proving it has reached a hard stop).
    const uint16_t MIN_TRAVEL   = 50;  // steps before stability check begins
    const uint16_t STABLE_TOL   = 4;   // noise floor in steps
    const int      STABLE_COUNT = 6;   // 300 ms of no movement

    // Read starting position before the sweep begins.
    ServoState st0{};
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_read_state(id, st0);
    xSemaphoreGive(bus_mutex);
    uint16_t start_pos = st0.position;
    Serial.printf("[cal]   id=%u start_pos=%u\n", id, start_pos);

    uint32_t phase_deadline = millis() + timeout_ms;
    uint16_t max_step = 4095;
    uint16_t prev_pos = start_pos;
    int      stable   = 0;
    bool     moving   = false;

    while (millis() < phase_deadline) {
        delay(50);
        monitor_record_cmd();
        ServoState st{};
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_read_state(id, st);
        xSemaphoreGive(bus_mutex);
        if (!ok) continue;

        if (!moving) {
            if ((uint16_t)abs((int)st.position - (int)start_pos) >= MIN_TRAVEL) {
                moving   = true;
                prev_pos = st.position;
                stable   = 0;
            }
            continue; // don't start counting until the servo has moved
        }

        if ((uint16_t)abs((int)st.position - (int)prev_pos) <= STABLE_TOL) {
            if (++stable >= STABLE_COUNT) {
                max_step = st.position;
                Serial.printf("[cal]   id=%u max_step=%u (stable)\n", id, max_step);
                break;
            }
        } else {
            stable   = 0;
            prev_pos = st.position;
        }
    }

    if (millis() >= phase_deadline) {
        Serial.printf("[cal]   id=%u timeout finding max end-stop\n", id);
        return r;
    }

    // Release end-stop: disable torque so the joint mechanically relaxes,
    // then re-enable. This clears the servo's internal overload-protection
    // flag before the reverse sweep begins.
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_write_torque(id, false);
    xSemaphoreGive(bus_mutex);
    delay(500);
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_write_torque(id, true);
    xSemaphoreGive(bus_mutex);
    delay(100);

    // -----------------------------------------------------------------------
    // Phase 2 — sweep toward min_step (negative direction)
    // -----------------------------------------------------------------------
    Serial.printf("[cal]   id=%u sweeping toward min...\n", id);

    // Read the actual position after the relaxation delay — the joint may
    // have drifted slightly from max_step while torque was off.
    ServoState st_relax{};
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_read_state(id, st_relax);
    xSemaphoreGive(bus_mutex);

    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_write_pos(id, 0, speed, 10);
    xSemaphoreGive(bus_mutex);

    start_pos = st_relax.position;
    Serial.printf("[cal]   id=%u phase2 start_pos=%u\n", id, start_pos);
    phase_deadline = millis() + timeout_ms;
    uint16_t min_step = 0;
    prev_pos = start_pos;
    stable   = 0;
    moving   = false;

    while (millis() < phase_deadline) {
        delay(50);
        monitor_record_cmd();
        ServoState st{};
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_read_state(id, st);
        xSemaphoreGive(bus_mutex);
        if (!ok) continue;

        if (!moving) {
            if ((uint16_t)abs((int)st.position - (int)start_pos) >= MIN_TRAVEL) {
                moving   = true;
                prev_pos = st.position;
                stable   = 0;
            }
            continue;
        }

        if ((uint16_t)abs((int)st.position - (int)prev_pos) <= STABLE_TOL) {
            if (++stable >= STABLE_COUNT) {
                min_step = st.position;
                Serial.printf("[cal]   id=%u min_step=%u (stable)\n", id, min_step);
                break;
            }
        } else {
            stable   = 0;
            prev_pos = st.position;
        }
    }

    if (millis() >= phase_deadline) {
        Serial.printf("[cal]   id=%u timeout finding min end-stop\n", id);
        return r;
    }

    // -----------------------------------------------------------------------
    // Phase 3 — compute calibration values and move to center
    // -----------------------------------------------------------------------
    if (max_step <= min_step) {
        Serial.printf("[cal]   id=%u bad result: max_step <= min_step\n", id);
        return r;
    }

    uint16_t center  = (uint16_t)(((uint32_t)min_step + max_step) / 2);
    float    range   = (max_step - min_step) / STEPS_PER_DEG;
    float    half    = range / 2.0f;
    float    safe_min = -(half - margin_deg);
    float    safe_max =  (half - margin_deg);

    // Update the live joint table
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (joints[i].id != id) continue;
        joints[i].center_step = center;
        joints[i].min_deg     = safe_min;
        joints[i].max_deg     = safe_max;
        break;
    }

    // Move to center
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_write_pos(id, center, speed, 10);
    xSemaphoreGive(bus_mutex);

    r.ok          = true;
    r.min_step    = min_step;
    r.max_step    = max_step;
    r.center_step = center;
    r.range_deg   = range;
    r.min_deg     = safe_min;
    r.max_deg     = safe_max;

    Serial.printf("[cal]   id=%u done: center=%u range=%.1f° limits=[%.1f, %.1f]\n",
                  id, center, range, safe_min, safe_max);
    return r;
}
