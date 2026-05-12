#include "monitor.h"
#include "joints.h"
#include "feetech.h"
#include "can_bus.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>

// ---------------------------------------------------------------------------
// Private state
// ---------------------------------------------------------------------------

static AsyncWebSocket    *_ws       = nullptr;
static SemaphoreHandle_t  _bus_mtx  = nullptr;
static volatile uint32_t  _last_cmd_ms = 0;
static bool               _torque_cut = false; // true once watchdog has fired
static bool               _sd_ok      = false;
static File               _log_file;

// ---------------------------------------------------------------------------
// SD / blackbox helpers
// ---------------------------------------------------------------------------

static void sd_init() {
    SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[sd]   no card detected — logging disabled");
        return;
    }
    _log_file = SD.open(SD_LOG_FILE, FILE_APPEND);
    if (!_log_file) {
        Serial.println("[sd]   failed to open log file");
        return;
    }
    _sd_ok = true;
    Serial.printf("[sd]   blackbox ready: %s\n", SD_LOG_FILE);
}

// Write one log line: "<millis> <level> <msg>\n"
static void sd_log(const char *level, const char *msg) {
    if (!_sd_ok) return;
    _log_file.printf("%lu %s %s\n", (unsigned long)millis(), level, msg);
    _log_file.flush();
}

static void sd_log_json(const char *level, JsonDocument &doc) {
    if (!_sd_ok) return;
    String s;
    serializeJson(doc, s);
    _log_file.printf("%lu %s %s\n", (unsigned long)millis(), level, s.c_str());
    _log_file.flush();
}

// ---------------------------------------------------------------------------
// Broadcast helpers
// ---------------------------------------------------------------------------

static void ws_broadcast(JsonDocument &doc) {
    if (!_ws) return;
    String out;
    serializeJson(doc, out);
    _ws->textAll(out);
}

static void emit_alert(uint8_t id, const char *reason, int value = -1) {
    JsonDocument doc;
    doc["type"]   = "alert";
    doc["ts"]     = millis();
    doc["id"]     = id;
    doc["reason"] = reason;
    if (value >= 0) doc["value"] = value;
    ws_broadcast(doc);
    sd_log_json("ALERT", doc);

    // Mirror alert onto CAN bus so peer nodes are informed
    CanAlertReason r = CAN_ALERT_TEMP; // default
    if      (strcmp(reason, "temp")     == 0) r = CAN_ALERT_TEMP;
    else if (strcmp(reason, "load")     == 0) r = CAN_ALERT_LOAD;
    else if (strcmp(reason, "volt")     == 0) r = CAN_ALERT_VOLT;
    else if (strcmp(reason, "watchdog") == 0) r = CAN_ALERT_WATCHDOG;
    else if (strcmp(reason, "estop")    == 0) r = CAN_ALERT_ESTOP;
    can_alert(id, r, (int16_t)(value >= 0 ? value : 0));

    Serial.printf("[alert] id=%u reason=%s", id, reason);
    if (value >= 0) Serial.printf(" value=%d", value);
    Serial.println();
}

// ---------------------------------------------------------------------------
// Emergency torque cut
// ---------------------------------------------------------------------------

static void cut_torque(const char *reason) {
    if (_torque_cut) return; // only fire once per watchdog cycle
    _torque_cut = true;

    uint8_t ids[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) ids[i] = joints[i].id;

    xSemaphoreTake(_bus_mtx, portMAX_DELAY);
    feetech_torque_all(ids, SERVO_COUNT, false);
    xSemaphoreGive(_bus_mtx);

    emit_alert(0, reason);
    sd_log("CRITICAL", reason);
    Serial.printf("[monitor] torque cut: %s\n", reason);
}

// ---------------------------------------------------------------------------
// Monitor task
// ---------------------------------------------------------------------------

static void monitor_task(void *) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_MS));

        // --- Watchdog check ---
        if (WATCHDOG_MS > 0) {
            uint32_t now     = millis();
            uint32_t elapsed = now - _last_cmd_ms;
            if (elapsed > WATCHDOG_MS && !_torque_cut) {
                cut_torque("watchdog");
            }
        }

        // --- Read all servo states ---
        JsonDocument telem;
        telem["type"] = "telemetry";
        telem["ts"]   = millis();
        JsonArray arr = telem["servos"].to<JsonArray>();

        for (int i = 0; i < SERVO_COUNT; i++) {
            uint8_t id = joints[i].id;
            ServoState st{};

            xSemaphoreTake(_bus_mtx, portMAX_DELAY);
            bool ok = feetech_read_state(id, st);
            xSemaphoreGive(_bus_mtx);

            JsonObject o = arr.add<JsonObject>();
            o["id"] = id;

            if (!ok) {
                o["err"] = "no_response";
                continue;
            }

            float angle = 0.0f;
            step_to_angle(id, st.position, &angle);

            o["pos"]   = st.position;
            o["angle"] = serialized(String(angle, 2));
            o["spd"]   = st.speed;
            o["load"]  = st.load;
            o["volt"]  = st.voltage;
            o["temp"]  = st.temperature;

            // --- Threshold checks ---
            if (st.temperature >= TEMP_LIMIT_C) {
                cut_torque("temp");
                emit_alert(id, "temp", st.temperature);
            }
            if (abs(st.load) > LOAD_LIMIT) {
                emit_alert(id, "load", (int)st.load);
            }
            if (st.voltage > 0 && st.voltage < VOLT_MIN) {
                emit_alert(id, "volt", st.voltage);
            }

            // Broadcast per-servo telemetry on CAN
            can_telemetry(id, st.position, st.load, st.temperature, st.voltage);
        }

        ws_broadcast(telem);
        sd_log_json("TELEM", telem);

        // CAN heartbeat — flags reflect current node health
        uint8_t flags = 0;
        if (!_torque_cut)       flags |= (1 << 0); // torque_ok
        if (_sd_ok)             flags |= (1 << 2); // sd_ok
        if (WiFi.status() == WL_CONNECTED) flags |= (1 << 3); // wifi_ok
        can_heartbeat(flags);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void monitor_start(AsyncWebSocket *ws_handle, SemaphoreHandle_t bus_mutex) {
    _ws      = ws_handle;
    _bus_mtx = bus_mutex;
    _last_cmd_ms = millis();

    sd_init();

    sd_log("INFO", "SO-101 monitor started");

    xTaskCreate(
        monitor_task,
        "monitor",
        4096,   // stack in bytes
        nullptr,
        1,      // priority (lower than default Arduino task at 1, so WS cmds preempt)
        nullptr
    );
}

void monitor_record_cmd() {
    _last_cmd_ms = millis();
    _torque_cut  = false; // re-arm watchdog after a command re-establishes contact
}

bool monitor_sd_ok() {
    return _sd_ok;
}
