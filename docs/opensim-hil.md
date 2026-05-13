# OpenSim HIL Integration

Hardware-In-the-Loop bridge between the SO-101 ESP32 firmware and the
[OpenSim](../../OpenSim/) browser simulation engine.

## Architecture

```
Browser (OpenSim)
  └── core/hil.js ──STATE_PATCH (armJoints)──> hub.js (port 3000)
                                                    │
                                          server/robot_bridge.js
                                                    │
                                 ┌──────────────────┴────────────────────┐
                                 │  sync_move / telemetry                │
                                 ▼                                       │
                           ESP32 (T-CAN485)                              │
                           WebSocket :80/ws                              │
                                 │                                       │
                       Feetech servo bus                                 │
                                 │                              telemetry STATE_PATCH
                        Physical arm moves ──────────────────────────────┘
```

The ESP32 remains the safety boundary in HIL mode.  Every `sync_move` issued by
the bridge still passes through FK self-collision checking, joint-limit clamping,
temperature/load/watchdog monitoring, and SD blackbox logging.

## Quick start

**1. Configure the bridge**

```bash
export HUB_URL=ws://localhost:3000   # or ws://<pi-ip>:3000
export ESP_URL=ws://192.168.1.100/ws  # ESP32 address from serial monitor
export ROOM=so101-hil
export SPEED=200                      # servo speed, steps/s
```

**2. Start hub + bridge**

```bash
cd OpenSim
node server/hub.js &
node server/robot_bridge.js
```

**3. Open OpenSim in a browser and join the HIL room**

```js
// In the browser console or via the OpenSim UI:
import { hilInit } from './core/hil.js';
hilInit('ws://localhost:3000', 'so101-hil');
```

Once connected, `S.armJoints` drives the physical arm and the ESP32's telemetry
updates `S.armJoints` back in real time — the 3-D model tracks the physical arm.

## State mapping

| OpenSim state key | Direction | Description |
|-------------------|-----------|-------------|
| `S.armJoints[0..4]` | Sim → Hardware | Joint angles in degrees (shoulder_pan … wrist_roll) |
| `S.armGripper` | Sim → Hardware | 0 = open, 1 = closed (maps to joint 5 angle 0° / 90°) |
| `S.armJoints[0..4]` | Hardware → Sim | Actual measured angles from servo telemetry |
| `S.armGripper` | Hardware → Sim | Actual gripper state |
| `S.armTemp[0..5]` | Hardware → Sim | Per-servo temperature (°C) |
| `S.armLoad[0..5]` | Hardware → Sim | Per-servo load (0–1000) |
| `S.armVolt[0..5]` | Hardware → Sim | Per-servo bus voltage (tenths of V) |
| `S.armAlert` | Hardware → Sim | `{id, reason, value}` from ESP32 alert events |

## Timing

| Parameter | Value |
|-----------|-------|
| Sim → Hardware tick | 50 Hz (configurable via `TICK_HZ` in `robot_bridge.js`) |
| Hardware → Sim telemetry | 2 Hz (ESP32 `TELEMETRY_MS=500` in `config.h`) |
| ESP32 watchdog | 2 s (cuts torque if no command arrives) |

The bridge sends commands at 50 Hz.  The ESP32 watchdog requires a command at
least every `WATCHDOG_MS` (default 2 s); the 50 Hz tick keeps the watchdog fed
as long as the bridge is running.

## Safety in HIL mode

- **Joint limits** — angle commands are clamped by `angle_to_step()` before being
  written to the servo.  Out-of-range commands are silently clipped.
- **FK check** — `sync_move` passes through `fk_check()`.  Commands that would
  cause self-collision or exceed the workspace envelope are rejected and logged.
  The bridge receives `{"ok":false,"err":"fk_reject","fk":"..."}` and drops the move.
- **Watchdog** — if the bridge disconnects, the ESP32 cuts torque after `WATCHDOG_MS`.
- **Temperature / load / voltage** — threshold violations trigger alerts regardless
  of the command source; the bridge forwards them to OpenSim as `armAlert` patches.

## Shared kinematic model

`src/kinematics.cpp` (firmware FK) and `display/robot_arm.js` (OpenSim renderer)
both model the SO-101 based on the same URDF geometry (`so101_new_calib.urdf`).
In HIL mode, the renderer shows the actual servo positions (from telemetry) rather
than commanded positions, so joint-limit clamps and FK rejections are immediately
visible as the sim and physical arm diverge.

## File reference

| File | Purpose |
|------|---------|
| `OpenSim/server/robot_bridge.js` | Hub ↔ ESP32 bridge (run on Pi or laptop) |
| `OpenSim/core/hil.js` | Browser HIL tick (extended for robot arm) |
| `OpenSim/display/robot_arm.js` | SO-101 3-D renderer (reads `S.armJoints`) |
| `SO101/src/kinematics.cpp` | FK + self-collision check on ESP32 |
| `SO101/src/config.h` | `FK_MIN_HEIGHT_M`, `FK_MAX_REACH_M` tuning |
