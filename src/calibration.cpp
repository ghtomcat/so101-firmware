#include "calibration.h"
#include "feetech.h"
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

    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    // Command a target far in the positive direction; the servo will stall
    // at the physical end-stop. We monitor load to detect the stall.
    feetech_write_pos(id, 4095, speed, 10);
    xSemaphoreGive(bus_mutex);

    uint32_t phase_deadline = millis() + timeout_ms;
    uint16_t max_step = 4095;

    while (millis() < phase_deadline) {
        delay(20);
        ServoState st{};
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_read_state(id, st);
        xSemaphoreGive(bus_mutex);

        if (!ok) continue;

        if (abs(st.load) >= (int16_t)load_thresh) {
            max_step = st.position;
            Serial.printf("[cal]   id=%u max_step=%u (load=%d)\n", id, max_step, st.load);
            break;
        }
    }

    if (millis() >= phase_deadline) {
        Serial.printf("[cal]   id=%u timeout finding max end-stop\n", id);
        return r; // ok stays false
    }

    delay(100); // brief pause before reversing

    // -----------------------------------------------------------------------
    // Phase 2 — sweep toward min_step (negative direction)
    // -----------------------------------------------------------------------
    Serial.printf("[cal]   id=%u sweeping toward min...\n", id);

    xSemaphoreTake(bus_mutex, portMAX_DELAY);
    feetech_write_pos(id, 0, speed, 10);
    xSemaphoreGive(bus_mutex);

    phase_deadline = millis() + timeout_ms;
    uint16_t min_step = 0;

    while (millis() < phase_deadline) {
        delay(20);
        ServoState st{};
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_read_state(id, st);
        xSemaphoreGive(bus_mutex);

        if (!ok) continue;

        if (abs(st.load) >= (int16_t)load_thresh) {
            min_step = st.position;
            Serial.printf("[cal]   id=%u min_step=%u (load=%d)\n", id, min_step, st.load);
            break;
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
