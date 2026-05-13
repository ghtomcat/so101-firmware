#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/semphr.h"

#include "config.h"
#include "feetech.h"
#include "joints.h"
#include "kinematics.h"
#include "monitor.h"
#include "calibration.h"
#include "can_bus.h"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

AsyncWebServer    server(80);
AsyncWebSocket    ws("/ws");

// Mutex that serialises all feetech_* calls. Used by both the WebSocket
// command handler and the background monitor task.
SemaphoreHandle_t bus_mutex;

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------

static void ws_send(AsyncWebSocketClient *client, JsonDocument &doc) {
    String out;
    serializeJson(doc, out);
    client->text(out);
}

static void reply_ok(AsyncWebSocketClient *client) {
    JsonDocument doc;
    doc["ok"] = true;
    ws_send(client, doc);
}

static void reply_error(AsyncWebSocketClient *client, const char *msg) {
    JsonDocument doc;
    doc["ok"]  = false;
    doc["err"] = msg;
    ws_send(client, doc);
}

static void reply_fk_reject(AsyncWebSocketClient *client, const char *reason) {
    JsonDocument doc;
    doc["ok"]  = false;
    doc["err"] = "fk_reject";
    doc["fk"]  = reason;
    ws_send(client, doc);
}

// ---------------------------------------------------------------------------
// Command dispatch
//
// Every command that reaches a servo resets the watchdog via monitor_record_cmd().
//
// Angle vs raw-step selection:
//   Commands accept "angle" (degrees, preferred) OR "pos" (raw 0–4095, escape
//   hatch for LeRobot and calibration tools). If both are present, "angle"
//   wins. If neither is present, the servo moves to its default_deg position.
// ---------------------------------------------------------------------------

static void handle_command(AsyncWebSocketClient *client, const String &raw) {
    JsonDocument req;
    if (deserializeJson(req, raw) != DeserializationError::Ok) {
        reply_error(client, "invalid json");
        return;
    }

    const char *cmd = req["cmd"] | "";

    // -----------------------------------------------------------------------
    // ping — check if a single servo is alive
    // Request:  { "cmd":"ping", "id":1 }
    // Response: { "ok":true,  "id":1 }
    //        or { "ok":false, "id":1 }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "ping") == 0) {
        uint8_t id = req["id"] | 1;
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_ping(id);
        xSemaphoreGive(bus_mutex);
        JsonDocument res;
        res["ok"] = ok;
        res["id"] = id;
        ws_send(client, res);
        if (ok) monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // scan — ping all joints and report which IDs responded
    // Request:  { "cmd":"scan" }
    // Response: { "ok":true, "found":[1,2,3,4,5,6] }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "scan") == 0) {
        JsonDocument res;
        res["ok"] = true;
        JsonArray found = res["found"].to<JsonArray>();
        for (int i = 0; i < SERVO_COUNT; i++) {
            uint8_t id = joints[i].id;
            xSemaphoreTake(bus_mutex, portMAX_DELAY);
            bool alive = feetech_ping(id);
            xSemaphoreGive(bus_mutex);
            if (alive) found.add(id);
        }
        ws_send(client, res);
        monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // debug_scan — raw bus diagnostic: captures RX bytes for each servo ping
    // Request:  { "cmd":"debug_scan" }
    // Response: { "ok":true, "servos":[
    //   {"id":1, "rx":0, "hex":"",         "valid":false},  <- nothing back
    //   {"id":2, "rx":6, "hex":"FF FF ...", "valid":true }   <- good response
    // ]}
    // rx=0  → no signal at all (check wiring / DIR pin / power)
    // rx>0  → servo is responding; "valid":true means checksum passed
    // Also prints raw bytes to the serial monitor for each ID.
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "debug_scan") == 0) {
        JsonDocument res;
        res["ok"] = true;
        JsonArray servos = res["servos"].to<JsonArray>();
        for (int i = 0; i < SERVO_COUNT; i++) {
            uint8_t id = joints[i].id;
            uint8_t rx[32];
            xSemaphoreTake(bus_mutex, portMAX_DELAY);
            int n = feetech_debug_ping(id, rx, sizeof(rx));
            xSemaphoreGive(bus_mutex);

            // Build hex string "FF FF 01 02 00 F9"
            char hex[3 * 32 + 1] = {};
            for (int j = 0; j < n; j++)
                snprintf(hex + j * 3, 4, "%02X ", rx[j]);
            if (n > 0) hex[n * 3 - 1] = '\0';

            // Valid ping response: FF FF id 02 00 chk
            uint8_t expected_chk = (uint8_t)~(uint8_t)(id + 2);
            bool valid = (n >= 6 && rx[0] == 0xFF && rx[1] == 0xFF &&
                          rx[2] == id && rx[3] == 0x02 && rx[4] == 0x00 &&
                          rx[5] == expected_chk);

            JsonObject sv = servos.add<JsonObject>();
            sv["id"]    = id;
            sv["rx"]    = n;
            sv["hex"]   = hex;
            sv["valid"] = valid;
        }
        ws_send(client, res);
        return;
    }

    // -----------------------------------------------------------------------
    // move — move a single joint
    // Request:  { "cmd":"move", "id":1, "angle":45.0, "speed":500, "acc":50 }
    //        or { "cmd":"move", "id":1, "pos":2560,   "speed":500, "acc":50 }
    // Response: { "ok":true }
    //        or { "ok":false, "err":"unknown joint" }
    //
    // speed: steps/s [0–4095], acc: [0–254]. Both are optional (defaults used).
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "move") == 0) {
        uint8_t  id    = req["id"]    | 1;
        uint16_t speed = req["speed"] | 500;
        uint8_t  acc   = req["acc"]   | 50;

        const JointConfig *j = joint_by_id(id);
        if (!j) { reply_error(client, "unknown joint"); return; }

        // Resolve target: "angle" wins over "pos" over default_deg.
        float    target_deg = j->default_deg;
        uint16_t step;

        if (!req["angle"].isNull()) {
            target_deg = req["angle"].as<float>();
            if (!angle_to_step(id, target_deg, &step)) {
                reply_error(client, "unknown joint");
                return;
            }
        } else if (!req["pos"].isNull()) {
            step = req["pos"].as<uint16_t>();
            step_to_angle(id, step, &target_deg);
        } else {
            angle_to_step(id, target_deg, &step);
        }

        // FK safety check: overlay this joint's new angle on current cached pose.
        float angles[SERVO_COUNT];
        monitor_get_angles(angles);
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (joints[i].id == id) { angles[i] = target_deg; break; }
        }
        char fk_reason[32];
        if (!fk_check(angles, fk_reason)) {
            reply_fk_reject(client, fk_reason);
            return;
        }

        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_write_pos(id, step, speed, acc);
        xSemaphoreGive(bus_mutex);

        JsonDocument res;
        res["ok"] = ok;
        ws_send(client, res);
        if (ok) monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // move_home — move all joints to their default_deg position
    // Request:  { "cmd":"move_home", "speed":300, "acc":30 }
    // Response: { "ok":true }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "move_home") == 0) {
        uint16_t speed = req["speed"] | 300;
        uint8_t  acc   = req["acc"]   | 30;

        uint8_t  ids[SERVO_COUNT], accs[SERVO_COUNT];
        uint16_t pos[SERVO_COUNT], spd[SERVO_COUNT];

        for (int i = 0; i < SERVO_COUNT; i++) {
            ids[i] = joints[i].id;
            accs[i] = acc;
            spd[i]  = speed;
            angle_to_step(joints[i].id, joints[i].default_deg, &pos[i]);
        }

        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        feetech_sync_write_pos(ids, pos, spd, accs, SERVO_COUNT);
        xSemaphoreGive(bus_mutex);

        reply_ok(client);
        monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // read — read full state of a single servo
    // Request:  { "cmd":"read", "id":1 }
    // Response: { "ok":true, "id":1, "pos":2048, "angle":0.00,
    //             "spd":0, "load":12, "volt":74, "temp":35 }
    //   volt unit: tenths of volts (74 = 7.4 V)
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "read") == 0) {
        uint8_t id = req["id"] | 1;
        ServoState st{};
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_read_state(id, st);
        xSemaphoreGive(bus_mutex);

        JsonDocument res;
        res["ok"] = ok;
        if (ok) {
            float angle = 0.0f;
            step_to_angle(id, st.position, &angle);
            res["id"]    = id;
            res["pos"]   = st.position;
            res["angle"] = serialized(String(angle, 2));
            res["spd"]   = st.speed;
            res["load"]  = st.load;
            res["volt"]  = st.voltage;
            res["temp"]  = st.temperature;
        }
        ws_send(client, res);
        monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // sync_move — move multiple joints simultaneously
    // Request:  { "cmd":"sync_move", "speed":500, "acc":50,
    //             "servos":[{"id":1,"angle":45.0},{"id":2,"angle":-30.0},...] }
    // Per-servo "speed" and "acc" override the top-level defaults if present.
    // Response: { "ok":true }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "sync_move") == 0) {
        JsonArray servos = req["servos"].as<JsonArray>();
        if (!servos) { reply_error(client, "missing servos array"); return; }

        uint16_t default_speed = req["speed"] | 500;
        uint8_t  default_acc   = req["acc"]   | 50;

        uint8_t  ids[SERVO_COUNT], accs[SERVO_COUNT];
        uint16_t pos[SERVO_COUNT], spd[SERVO_COUNT];
        float    degs[SERVO_COUNT];
        uint8_t  count = 0;

        for (JsonObject s : servos) {
            if (count >= SERVO_COUNT) break;
            uint8_t  id   = s["id"] | 1;
            uint16_t step = 0;
            float    deg  = 0.0f;

            const JointConfig *j = joint_by_id(id);
            if (!j) continue;

            if (!s["angle"].isNull()) {
                deg = s["angle"].as<float>();
                if (!angle_to_step(id, deg, &step)) continue;
            } else if (!s["pos"].isNull()) {
                step = s["pos"].as<uint16_t>();
                step_to_angle(id, step, &deg);
            } else {
                deg = j->default_deg;
                angle_to_step(id, deg, &step);
            }

            ids[count]  = id;
            pos[count]  = step;
            spd[count]  = s["speed"] | default_speed;
            accs[count] = s["acc"]   | default_acc;
            degs[count] = deg;
            count++;
        }

        if (count == 0) { reply_error(client, "no valid servos"); return; }

        // FK safety check: overlay all commanded angles on current cached pose.
        float angles[SERVO_COUNT];
        monitor_get_angles(angles);
        for (uint8_t n = 0; n < count; n++) {
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (joints[i].id == ids[n]) { angles[i] = degs[n]; break; }
            }
        }
        char fk_reason[32];
        if (!fk_check(angles, fk_reason)) {
            reply_fk_reject(client, fk_reason);
            return;
        }

        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        feetech_sync_write_pos(ids, pos, spd, accs, count);
        xSemaphoreGive(bus_mutex);

        reply_ok(client);
        monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // sync_read — read positions of multiple joints
    // Request:  { "cmd":"sync_read", "ids":[1,2,3,4,5,6] }
    // Response: { "ok":true, "servos":[{"id":1,"pos":2048,"angle":0.00},...] }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "sync_read") == 0) {
        JsonArray id_arr = req["ids"].as<JsonArray>();
        if (!id_arr) { reply_error(client, "missing ids array"); return; }

        uint8_t  ids[SERVO_COUNT];
        uint16_t pos[SERVO_COUNT];
        uint8_t  count = 0;
        for (JsonVariant v : id_arr) {
            if (count >= SERVO_COUNT) break;
            ids[count++] = v.as<uint8_t>();
        }

        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        bool ok = feetech_sync_read_pos(ids, pos, count);
        xSemaphoreGive(bus_mutex);

        JsonDocument res;
        res["ok"] = ok;
        JsonArray out = res["servos"].to<JsonArray>();
        if (ok) {
            for (int i = 0; i < count; i++) {
                float angle = 0.0f;
                step_to_angle(ids[i], pos[i], &angle);
                JsonObject o = out.add<JsonObject>();
                o["id"]    = ids[i];
                o["pos"]   = pos[i];
                o["angle"] = serialized(String(angle, 2));
            }
        }
        ws_send(client, res);
        monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // stop — immediately disable torque on all servos (emergency stop)
    // Request:  { "cmd":"stop" }
    // Response: { "ok":true }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "stop") == 0) {
        uint8_t ids[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) ids[i] = joints[i].id;
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        feetech_torque_all(ids, SERVO_COUNT, false);
        xSemaphoreGive(bus_mutex);
        reply_ok(client);
        // Intentionally do NOT call monitor_record_cmd() — stop is not a
        // motion command, so the watchdog stays armed.
        return;
    }

    // -----------------------------------------------------------------------
    // torque — enable or disable torque on all servos
    // Request:  { "cmd":"torque", "enable":true }
    // Response: { "ok":true }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "torque") == 0) {
        bool enable = req["enable"] | true;
        uint8_t ids[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) ids[i] = joints[i].id;
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        feetech_torque_all(ids, SERVO_COUNT, enable);
        xSemaphoreGive(bus_mutex);
        reply_ok(client);
        if (enable) monitor_record_cmd();
        return;
    }

    // -----------------------------------------------------------------------
    // config_joint — update a joint's center_step or direction at runtime
    // Useful during calibration without reflashing.
    // Request:  { "cmd":"config_joint", "id":1, "center":2048, "dir":1 }
    // Response: { "ok":true }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "config_joint") == 0) {
        uint8_t id = req["id"] | 1;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (joints[i].id != id) continue;
            if (!req["center"].isNull()) joints[i].center_step = req["center"].as<uint16_t>();
            if (!req["dir"].isNull())    joints[i].direction   = req["dir"].as<int8_t>();
            if (!req["min"].isNull())    joints[i].min_deg     = req["min"].as<float>();
            if (!req["max"].isNull())    joints[i].max_deg     = req["max"].as<float>();
            reply_ok(client);
            return;
        }
        reply_error(client, "unknown joint");
        return;
    }

    // -----------------------------------------------------------------------
    // calibrate — end-stop sweep for one joint
    // Blocks for up to 2 × timeout_ms while sweeping. Progress events are
    // broadcast to all WebSocket clients by the monitor task during the sweep.
    //
    // Request:  { "cmd":"calibrate", "id":1, "speed":150, "margin_deg":5.0,
    //             "load_thresh":400, "timeout_ms":8000 }
    // Response: { "ok":true, "id":1, "center":2048, "min_deg":-82.5,
    //             "max_deg":82.5, "range_deg":170.0,
    //             "min_step":1093, "max_step":3003 }
    //        or { "ok":false, "err":"end-stop not found" }
    //
    // After a successful sweep the result is saved to NVS automatically.
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "calibrate") == 0) {
        uint8_t  id          = req["id"]          | 1;
        uint16_t speed       = req["speed"]       | 150;
        float    margin      = req["margin_deg"]  | 5.0f;
        uint16_t load_thresh = req["load_thresh"] | 400;
        uint32_t timeout_ms  = req["timeout_ms"]  | 8000;

        // cal_sweep blocks — run it here in the WS callback task.
        // The sweep takes the bus_mutex internally so the monitor task backs off.
        CalResult r = cal_sweep(id, speed, margin, load_thresh, timeout_ms);

        if (!r.ok) {
            reply_error(client, "end-stop not found");
            return;
        }

        // Persist to NVS
        cal_save();
        monitor_record_cmd();

        JsonDocument res;
        res["ok"]        = true;
        res["id"]        = r.id;
        res["center"]    = r.center_step;
        res["min_deg"]   = serialized(String(r.min_deg,   2));
        res["max_deg"]   = serialized(String(r.max_deg,   2));
        res["range_deg"] = serialized(String(r.range_deg, 2));
        res["min_step"]  = r.min_step;
        res["max_step"]  = r.max_step;
        ws_send(client, res);
        return;
    }

    // -----------------------------------------------------------------------
    // cal_erase — wipe NVS calibration, revert to firmware defaults
    // Request:  { "cmd":"cal_erase" }
    // Response: { "ok":true }
    // -----------------------------------------------------------------------
    if (strcmp(cmd, "cal_erase") == 0) {
        cal_erase();
        reply_ok(client);
        return;
    }

    reply_error(client, "unknown command");
}

// ---------------------------------------------------------------------------
// WebSocket event handler
// ---------------------------------------------------------------------------

static void on_ws_event(AsyncWebSocket *, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[ws] client #%u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
        JsonDocument info;
        info["type"]    = "connected";
        info["sd_log"]  = monitor_sd_ok();
        info["joints"]  = SERVO_COUNT;
        String out;
        serializeJson(info, out);
        client->text(out);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[ws] client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len &&
            info->opcode == WS_TEXT) {
            String msg((char *)data, len);
            handle_command(client, msg);
        }
    }
}

// ---------------------------------------------------------------------------
// Setup & loop
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[SO-101] booting...");

    // --- T-CAN485 board power pins ---
    pinMode(PIN_5V_EN,    OUTPUT);
    digitalWrite(PIN_5V_EN,    HIGH); // power the RS-485 chip

    pinMode(RS485_SE_PIN, OUTPUT);
    digitalWrite(RS485_SE_PIN, HIGH); // full slew rate (required at 1 Mbps)

    // --- Servo bus mutex ---
    bus_mutex = xSemaphoreCreateMutex();

    // --- Feetech servo driver ---
    feetech_init(Serial2, SERVO_RX_PIN, SERVO_TX_PIN, SERVO_DIR_PIN, SERVO_BAUD);
    Serial.println("[servo] UART ready");

    // --- Load calibration from NVS (overlays firmware defaults in joints[]) ---
    cal_load();

    // --- WiFi ---
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[wifi]  connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[wifi]  connected: %s\n", WiFi.localIP().toString().c_str());

    // --- CAN bus (disabled at compile time when CAN_ENABLE=0) ---
    can_init();

    // Route incoming CAN CMD_MOVE through the same FK check and servo path.
    can_set_move_callback([](uint8_t sid, uint16_t step, uint16_t spd, uint8_t acc) {
        float deg = 0.0f;
        step_to_angle(sid, step, &deg);
        float angles[SERVO_COUNT];
        monitor_get_angles(angles);
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (joints[i].id == sid) { angles[i] = deg; break; }
        }
        char reason[32];
        if (!fk_check(angles, reason)) {
            Serial.printf("[fk]   CAN move rejected id=%u: %s\n", sid, reason);
            return;
        }
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        feetech_write_pos(sid, step, spd, acc);
        xSemaphoreGive(bus_mutex);
        monitor_record_cmd();
    });

    // Route incoming CAN CMD_STOP to the same torque-cut path.
    can_set_stop_callback([]() {
        uint8_t ids[SERVO_COUNT];
        for (int i = 0; i < SERVO_COUNT; i++) ids[i] = joints[i].id;
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
        feetech_torque_all(ids, SERVO_COUNT, false);
        xSemaphoreGive(bus_mutex);
    });

    // --- Background monitor (telemetry + watchdog + SD blackbox) ---
    monitor_start(&ws, bus_mutex);

    // --- WebSocket + HTTP server ---
    ws.onEvent(on_ws_event);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/plain", "SO-101 OK");
    });

    server.begin();
    Serial.println("[http]  server started");
    Serial.printf("[ws]    ws://%s/ws\n", WiFi.localIP().toString().c_str());
}

void loop() {
    ws.cleanupClients();
    delay(50);
}
