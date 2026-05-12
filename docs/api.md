# SO-101 ESP32 Firmware — API Reference

The firmware exposes a **WebSocket endpoint** at `ws://<device-ip>/ws`.  
A plain HTTP health-check is available at `http://<device-ip>/`.

All messages are UTF-8 JSON. Every response includes `"ok": true | false`.  
On error, a `"err"` string is also present.

---

## Connection

When a client connects, the firmware immediately sends a welcome frame:

```json
{ "type": "connected", "sd_log": true, "joints": 6 }
```

`sd_log` is `true` if an SD card was detected at boot and the blackbox log is active.

---

## Commands

### `ping` — Check a single servo

```json
// Request
{ "cmd": "ping", "id": 1 }

// Response
{ "ok": true,  "id": 1 }
{ "ok": false, "id": 1 }
```

---

### `scan` — Discover all joints

Pings servo IDs 1–6 and reports which ones respond.

```json
// Request
{ "cmd": "scan" }

// Response
{ "ok": true, "found": [1, 2, 3, 4, 5, 6] }
```

---

### `move` — Move a single joint

Accepts **degrees** (`"angle"`) or raw steps (`"pos"`, 0–4095).  
If neither is present, the joint moves to its configured `default_deg`.  
`"angle"` is clamped to the joint's `[min_deg, max_deg]` limits.

| Field   | Type  | Default | Description                        |
|---------|-------|---------|------------------------------------|
| `id`    | int   | 1       | Servo ID (1–6)                     |
| `angle` | float | —       | Target angle in degrees (preferred)|
| `pos`   | int   | —       | Raw step value 0–4095 (escape hatch)|
| `speed` | int   | 500     | Speed in steps/s (0–4095)          |
| `acc`   | int   | 50      | Acceleration (0–254)               |

```json
// Request (angle)
{ "cmd": "move", "id": 1, "angle": 45.0, "speed": 500, "acc": 50 }

// Request (raw step — for LeRobot / calibration tools)
{ "cmd": "move", "id": 1, "pos": 2560 }

// Response
{ "ok": true }
```

---

### `move_home` — Move all joints to home position

Sends all joints to their `default_deg` simultaneously.

```json
// Request
{ "cmd": "move_home", "speed": 300, "acc": 30 }

// Response
{ "ok": true }
```

---

### `read` — Read full state of one servo

```json
// Request
{ "cmd": "read", "id": 1 }

// Response
{
  "ok":    true,
  "id":    1,
  "pos":   2048,
  "angle": "0.00",
  "spd":   0,
  "load":  12,
  "volt":  74,
  "temp":  35
}
```

| Field   | Unit          | Notes                              |
|---------|---------------|------------------------------------|
| `pos`   | steps 0–4095  | Raw servo position                 |
| `angle` | degrees       | Converted using joint calibration  |
| `spd`   | steps/s       | Signed; bit 15 = direction         |
| `load`  | 0–1000        | % of stall torque × 10; signed     |
| `volt`  | tenths of V   | e.g. `74` = 7.4 V                  |
| `temp`  | °C            |                                    |

---

### `sync_move` — Move multiple joints simultaneously

Top-level `speed` and `acc` are defaults; per-servo values override them.

```json
// Request
{
  "cmd":   "sync_move",
  "speed": 500,
  "acc":   50,
  "servos": [
    { "id": 1, "angle":  45.0 },
    { "id": 2, "angle": -30.0 },
    { "id": 3, "angle":  90.0, "speed": 300 }
  ]
}

// Response
{ "ok": true }
```

---

### `sync_read` — Read positions of multiple joints

```json
// Request
{ "cmd": "sync_read", "ids": [1, 2, 3, 4, 5, 6] }

// Response
{
  "ok": true,
  "servos": [
    { "id": 1, "pos": 2048, "angle": "0.00" },
    { "id": 2, "pos": 2560, "angle": "45.00" }
  ]
}
```

---

### `stop` — Emergency stop (instant torque off)

Disables torque on all servos immediately via a single SYNC_WRITE.  
Does **not** reset the watchdog — send `torque` with `enable: true` to re-arm.

```json
// Request
{ "cmd": "stop" }

// Response
{ "ok": true }
```

---

### `torque` — Enable or disable torque on all servos

```json
// Request
{ "cmd": "torque", "enable": true }
{ "cmd": "torque", "enable": false }

// Response
{ "ok": true }
```

---

### `config_joint` — Update joint calibration at runtime

Changes take effect immediately without reflashing. To persist across reboots,
update `joints[]` in `src/joints.cpp` and reflash.

| Field    | Type  | Description                            |
|----------|-------|----------------------------------------|
| `id`     | int   | Servo ID to update                     |
| `center` | int   | Step value at 0° (0–4095)              |
| `dir`    | int   | +1 or -1 (physical mounting direction) |
| `min`    | float | Minimum allowed angle (degrees)        |
| `max`    | float | Maximum allowed angle (degrees)        |

```json
// Request
{ "cmd": "config_joint", "id": 1, "center": 2048, "dir": 1, "min": -170, "max": 170 }

// Response
{ "ok": true }
```

---

## Autonomous broadcasts

The firmware sends these frames without a request.

### `telemetry` — Periodic state of all servos

Broadcast every `TELEMETRY_MS` (default 500 ms) to all connected clients.

```json
{
  "type": "telemetry",
  "ts":   123456,
  "servos": [
    { "id":1, "pos":2048, "angle":"0.00", "spd":0, "load":12, "volt":74, "temp":35 },
    { "id":2, "pos":2560, "angle":"45.00", "spd":0, "load":8,  "volt":74, "temp":34 }
  ]
}
```

### `alert` — Threshold breach or watchdog

```json
{ "type": "alert", "ts": 123456, "id": 1, "reason": "temp",     "value": 72  }
{ "type": "alert", "ts": 123456, "id": 1, "reason": "load",     "value": 850 }
{ "type": "alert", "ts": 123456, "id": 1, "reason": "volt",     "value": 58  }
{ "type": "alert", "ts": 123456, "id": 0, "reason": "watchdog"               }
```

`id: 0` means the event applies to all servos (e.g. watchdog).  
A `temp` or `watchdog` alert means torque has already been cut.

---

## Safety behaviour

| Condition                        | Action                                  |
|----------------------------------|-----------------------------------------|
| `temp` ≥ `TEMP_LIMIT_C` (70 °C) | Torque cut + alert broadcast + SD log   |
| `load` > `LOAD_LIMIT` (800)      | Alert broadcast + SD log (no torque cut)|
| `volt` < `VOLT_MIN` (6.0 V)      | Alert broadcast + SD log (no torque cut)|
| No command for `WATCHDOG_MS` (2 s)| Torque cut + alert broadcast + SD log   |

Thresholds are defined in `src/config.h`.

---

## Blackbox (SD card log)

If a microSD card is present at boot, all telemetry and alerts are appended
to `/so101.log` in this format:

```
<millis> <LEVEL> <json-payload>
```

Example:

```
12345 TELEM {"type":"telemetry","ts":12345,...}
67890 ALERT {"type":"alert","ts":67890,"id":1,"reason":"temp","value":72}
67891 CRITICAL temp
```

The log file persists across reboots (opened in append mode).  
Levels: `INFO`, `TELEM`, `ALERT`, `CRITICAL`.

---

## Calibration

### `calibrate` — automated end-stop sweep

Moves the joint slowly to each physical end-stop, detects the stall via load
spike, computes the center and safe range, then saves the result to **ESP32
NVS flash**. The result is loaded automatically on every boot.

| Field         | Type   | Default | Description                                  |
|---------------|--------|---------|----------------------------------------------|
| `id`          | int    | 1       | Servo ID to calibrate                        |
| `speed`       | int    | 150     | Sweep speed in steps/s (keep ≤ 200)          |
| `margin_deg`  | float  | 5.0     | Safety buffer inside each end-stop (degrees) |
| `load_thresh` | int    | 400     | Load value that signals an end-stop (0–1000) |
| `timeout_ms`  | int    | 8000    | Max time to reach each end-stop (ms)         |

```json
// Request
{ "cmd": "calibrate", "id": 4, "speed": 150, "margin_deg": 5.0 }

// Response on success
{
  "ok": true, "id": 4,
  "center": 2231, "min_deg": "-82.50", "max_deg": "82.50",
  "range_deg": "170.00", "min_step": 1093, "max_step": 3003
}

// Response on failure (end-stop not reached within timeout)
{ "ok": false, "err": "end-stop not found" }
```

After a successful sweep the firmware:
1. Updates the live `joints[]` table immediately (limits enforced at once).
2. Saves center, direction, min and max to NVS — persists across reboots.
3. Returns the joint to its new center position.

### `cal_erase` — wipe calibration from NVS

Removes all saved calibration data. Firmware defaults from `src/joints.cpp`
take effect immediately and on next boot.

```json
{ "cmd": "cal_erase" }
// Response
{ "ok": true }
```

### `config_joint` — manual override

Use this if you need to adjust a single value without running a full sweep
(e.g. to flip a direction after re-mounting a servo). Changes are live
immediately but are **not** automatically saved to NVS — send `cal_erase`
first if you want a clean state, or follow up with a `calibrate` sweep to
re-derive everything from the physical stops.

### NVS storage layout

Namespace: `so101cal`

| Key      | Type    | Description              |
|----------|---------|--------------------------|
| `jN_cen` | uint32  | `center_step` for joint N |
| `jN_dir` | int32   | `direction` for joint N   |
| `jN_min` | float   | `min_deg` for joint N     |
| `jN_max` | float   | `max_deg` for joint N     |

Only joints that have been calibrated have NVS entries. Uncalibrated joints
use the firmware defaults from `src/joints.cpp`.

---

## CAN bus

The CAN layer is controlled by a single flag in `src/config.h`:

```c
#define CAN_ENABLE  1   // set to 0 to compile away the entire CAN subsystem
```

When `CAN_ENABLE` is `0` every CAN function becomes an empty inline. No code,
no pins, no TWAI driver — the binary is identical to a build that never
included CAN. There are no `#if` guards needed in application code.

---

### Configuration (`src/config.h`)

```c
#define CAN_NODE_ID         1     // this node's address on the bus (1–15)
#define CAN_BAUD_KBPS       500   // 125 | 250 | 500 | 1000 — wrong value = compile error
#define CAN_VOTE_QUORUM     2     // set to 1 to disable voting (single-node)
#define CAN_VOTE_TIMEOUT_MS 150   // max wait for peer votes (ms)
#define CAN_HEARTBEAT_MS    1000  // heartbeat interval (ms)
```

---

### Message ID scheme (11-bit standard frames)

Node-specific messages encode the sender's `node_id` in the low nibble.

| ID range         | Direction | Message      | Sent by             |
|------------------|-----------|--------------|---------------------|
| `0x101` – `0x10F`| broadcast | `HEARTBEAT`  | each node, 1 Hz     |
| `0x111` – `0x11F`| broadcast | `TELEMETRY`  | each node, per servo per cycle |
| `0x121` – `0x12F`| broadcast | `ALERT`      | each node, on event |
| `0x131` – `0x13F`| broadcast | `VOTE_REQ`   | requesting node     |
| `0x141` – `0x14F`| broadcast | `VOTE_ACK`   | responding node     |
| `0x200`          | broadcast | `CMD_MOVE`   | coordinator / any   |
| `0x201`          | broadcast | `CMD_STOP`   | coordinator / any   |

`0x100 + node_id`, `0x110 + node_id`, etc.

---

### Message formats

All multi-byte integers are **little-endian**.

#### `HEARTBEAT` — `0x100 + node_id`, 8 bytes

| Byte | Field          | Notes                                         |
|------|----------------|-----------------------------------------------|
| 0    | `node_id`      |                                               |
| 1    | `status_flags` | bit 0 = torque OK, bit 2 = SD OK, bit 3 = WiFi OK, bit 4 = alert active |
| 2–3  | `uptime_s`     | seconds since boot (uint16)                   |
| 4–7  | reserved       |                                               |

#### `TELEMETRY` — `0x110 + node_id`, 8 bytes, one frame per servo

| Byte | Field      | Notes                         |
|------|------------|-------------------------------|
| 0    | `node_id`  |                               |
| 1    | `servo_id` | 1–6                           |
| 2–3  | `step`     | raw position 0–4095 (uint16)  |
| 4–5  | `load`     | signed, 0–1000 = % stall torque (int16) |
| 6    | `temp`     | °C                            |
| 7    | `volt`     | tenths of volts               |

#### `ALERT` — `0x120 + node_id`, 8 bytes

| Byte | Field      | Notes                                                    |
|------|------------|----------------------------------------------------------|
| 0    | `node_id`  |                                                          |
| 1    | `servo_id` | 0 = applies to all servos                               |
| 2    | `reason`   | 0=temp, 1=load, 2=volt, 3=watchdog, 4=estop             |
| 3–4  | `value`    | measured value that caused the alert (int16)             |
| 5–7  | reserved   |                                                          |

#### `VOTE_REQ` — `0x130 + node_id`, 7 bytes

Sent by the initiating node to request quorum before executing a move.

| Byte | Field              | Notes                              |
|------|--------------------|------------------------------------|
| 0    | `requesting_node`  |                                    |
| 1    | `seq`              | sequence number (wraps at 255)     |
| 2    | `servo_id`         |                                    |
| 3–4  | `target_step`      | uint16                             |
| 5–6  | `speed`            | uint16                             |

#### `VOTE_ACK` — `0x140 + node_id`, 5 bytes

| Byte | Field             | Notes                                              |
|------|-------------------|----------------------------------------------------|
| 0    | `responding_node` |                                                    |
| 1    | `requesting_node` | echoed from VOTE_REQ                               |
| 2    | `seq`             | echoed from VOTE_REQ                               |
| 3    | `result`          | 1 = approve, 0 = reject                            |
| 4    | `reject_reason`   | 0 = out of range, 1 = alert active, 2 = torque off |

#### `CMD_MOVE` — `0x200`, 7 bytes

| Byte | Field         | Notes                                |
|------|---------------|--------------------------------------|
| 0    | `target_node` | node_id to move, `0xFF` = all nodes  |
| 1    | `servo_id`    |                                      |
| 2–3  | `step`        | raw target position (uint16)         |
| 4–5  | `speed`       | steps/s (uint16)                     |
| 6    | `acc`         | 0–254                                |

#### `CMD_STOP` — `0x201`, 2 bytes

| Byte | Field         | Notes                               |
|------|---------------|-------------------------------------|
| 0    | `target_node` | node_id to stop, `0xFF` = all nodes |
| 1    | reserved      |                                     |

---

### Voting flow (Airbus N-of-M)

The voting pattern is borrowed from the Airbus EFCS architecture where multiple
independent flight control computers must agree before an input is acted on.
Here each T-CAN485 node independently validates a proposed move against its own
safety state before the network approves it.

```
Initiating node                    Peer nodes
───────────────                    ──────────
broadcast VOTE_REQ ──────────────► each peer evaluates:
                                     • is target_step within this node's limits?
                                     • is this node currently in an alert state?
                                   send VOTE_ACK (approve / reject + reason)
collect ACKs ◄────────────────────
if approvals ≥ CAN_VOTE_QUORUM
  within CAN_VOTE_TIMEOUT_MS
    → execute move
else
    → abort, log rejection
```

A rejection from any single peer does not automatically block — only the
quorum count matters. This tolerates a single faulty or missing node (the
Byzantine generals problem reduced to a practical N-of-M threshold).

**Single-node operation** — set `CAN_VOTE_QUORUM 1`. The node self-approves
all moves without broadcasting any `VOTE_REQ` frames. Zero bus traffic, zero
latency penalty, same code path.

---

### Multi-node setup

Each arm is one node. Assign unique `CAN_NODE_ID` values (1–15) and reflash.
Wire all nodes to the same CAN bus with a 120 Ω termination resistor at each
physical end of the bus.

```
Node 1 (arm A)        Node 2 (arm B)        Node 3 (arm C)
T-CAN485              T-CAN485              T-CAN485
CAN H ────────────────CAN H ────────────────CAN H
CAN L ────────────────CAN L ────────────────CAN L
[120Ω]                                      [120Ω]
```

All nodes independently:
- Read their own servos and broadcast `TELEMETRY` at 500 ms
- Broadcast `HEARTBEAT` at 1 s
- Respond to `VOTE_REQ` from peers
- Execute `CMD_MOVE` / `CMD_STOP` when `target_node` matches their ID or is `0xFF`

A separate coordinator (PLC, Raspberry Pi, another ESP32) can issue `CMD_MOVE`
and `CMD_STOP` without participating in voting. It observes `ALERT` and
`HEARTBEAT` frames to monitor fleet health.

---

### What a peer rejection looks like

```
Node 2 is overheating → emits ALERT (reason=temp) → sets alert_active flag

Node 1 requests a sync move:
  → broadcasts VOTE_REQ for servo 3, target_step 2800
  → Node 2 sees VOTE_REQ, checks alert_active → sends VOTE_ACK reject (reason=alert_active)
  → Node 1 times out without quorum
  → move aborted on all nodes
  → Node 1 logs "vote timeout" to SD and WebSocket clients
```

This is the Airbus pattern: a health event on one channel vetoes the whole
network until the fault is cleared.

---

## Wiring (T-CAN485 → SO-101)

```
STS3215 pin    T-CAN485
────────────────────────────────────────
G  (GND)   ──  GND  +  RS-485 terminal B
V1 (VCC)   ──  External 7.4 V supply (not T-CAN485)
S  (Signal)──  RS-485 terminal A
```

Servos daisy-chain through their dual JST ports. All three signals (G, V1, S)
are shared across the whole chain.

Pin assignments (`src/config.h`):

| Signal         | GPIO | Notes                                        |
|----------------|------|----------------------------------------------|
| RS-485 TX      | 22   |                                              |
| RS-485 RX      | 21   |                                              |
| RS-485 DIR     | 17   | HIGH = transmit, LOW = receive               |
| RS-485 SE      | 19   | HIGH = full slew rate (required at 1 Mbps)   |
| 5 V enable     | 16   | HIGH to power RS-485 chip                    |
| SD MISO        | 2    |                                              |
| SD MOSI        | 15   |                                              |
| SD SCLK        | 14   |                                              |
| SD CS          | 13   |                                              |
| CAN TX         | 26   |                                              |
| CAN RX         | 27   |                                              |
| CAN SE         | 23   | LOW = TJA1050 active, HIGH = standby         |
