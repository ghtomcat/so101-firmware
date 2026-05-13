# so101-firmware

ESP32 firmware for the [Seeed Studio SO-101](https://www.seeedstudio.com/SO-ARM101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) robot arm, running on the [LILYGO T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485) board (Feetech STS3215 servos).

The defining design principle: **the ESP32 is the permanent safety boundary**. Every motion command — from a browser, a Python script, a LeRobot policy, or a CAN peer — passes through the same joint-limit, FK self-collision, watchdog, and blackbox layer before any servo moves. There is no bypass.

---

## Safety architecture

```
Physical end-stops
  └── Joint-limit clamping          angle_to_step() clips to [min°, max°]
        └── FK self-collision check   rejects poses before they reach the bus
              └── Torque watchdog      cuts power if no command for 2 s
                    └── Thermal / load monitor   per-servo alerts at threshold
                          └── SD blackbox    every command and rejection timestamped
```

The FK check computes full forward kinematics from URDF-derived constants entirely on the ESP32 — no offload to a host PC. Each proposed pose is tested against:

- **Floor plane** — no link node below Z = 0
- **Workspace sphere** — TCP within configurable reach radius
- **15 capsule pairs** — all non-adjacent link pairs checked for interpenetration

A rejected move returns a structured error; the servo bus is never written:

```json
{ "ok": false, "err": "fk_reject", "fk": "collision:2-5" }
{ "ok": false, "err": "fk_reject", "fk": "below_floor:3" }
{ "ok": false, "err": "fk_reject", "fk": "over_reach"    }
```

### CAN voting (Airbus EFCS pattern)

When multiple T-CAN485 nodes are connected, a proposed move is broadcast as a `VOTE_REQ` before execution. Each peer independently validates the move against its own safety state and replies with a `VOTE_ACK`. The move only executes if approvals ≥ `CAN_VOTE_QUORUM` within the timeout window — any node in an alert state vetoes the network until the fault clears.

This pattern is borrowed from Airbus fly-by-wire architecture (EFCS), where independent flight control computers must agree before an input is acted on.

---

## Hardware

| Component | Part |
|-----------|------|
| MCU board | [LILYGO T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485) (ESP32) |
| Arm | Seeed Studio SO-101 |
| Servos | Feetech STS3215 × 6 |
| Bus | RS-485 — terminal A → servo S pin, terminal B → GND |
| Power | External 7.4 V → servo V1 pins (not from the T-CAN485) |

The T-CAN485's onboard SP3485 transceiver handles half-duplex switching automatically.

---

## Quickstart

**1. Configure** `src/config.h`:

```c
#define WIFI_SSID   "your-network"
#define WIFI_PASS   "your-password"
#define CAN_ENABLE  0   // 1 for multi-node CAN setup
```

**2. Build and flash:**

```bash
pio run -t upload
pio device monitor   # prints the IP address once WiFi connects
```

**3. Send commands over WebSocket** (`ws://<device-ip>/ws`):

```js
const ws = new WebSocket("ws://<device-ip>/ws");

ws.send(JSON.stringify({ cmd: "scan" }));                          // discover servos
ws.send(JSON.stringify({ cmd: "calibrate", id: 1 }));             // auto end-stop sweep
ws.send(JSON.stringify({ cmd: "move", id: 1, angle: 45.0 }));     // move to 45°
ws.send(JSON.stringify({ cmd: "sync_move", speed: 500, acc: 50,
  servos: [{ id: 1, angle: 0 }, { id: 2, angle: 45 }] }));        // move multiple joints
ws.send(JSON.stringify({ cmd: "stop" }));                          // emergency stop
```

---

## Features

- **Angle-based control** — send degrees, not raw steps; calibration stored in NVS flash
- **Automated calibration** — end-stop sweep finds physical limits and computes joint center automatically
- **FK self-collision checking** — full forward kinematics on the ESP32 using URDF geometry
- **Joint limits** — clamped in firmware regardless of command source
- **Watchdog** — torque cut if no command arrives within 2 s
- **Thermal / load / voltage monitoring** — per-servo alerts with configurable thresholds
- **SD blackbox** — every telemetry frame, alert, and FK rejection logged to `/so101.log`
- **CAN bus voting** — Airbus-style N-of-M quorum for multi-node coordination
- **LeRobot compatible** — raw step escape hatch (`"pos"`) lets LeRobot talk directly; safety applies to all controllers equally
- **OpenSim HIL** — hardware-in-the-loop bridge to a browser-based 3-D simulator at 50 Hz

---

## Project structure

```
src/
  config.h          — WiFi, pins, safety thresholds, FK limits, CAN settings
  feetech.h/.cpp    — Feetech STS3215 half-duplex UART driver
  joints.h/.cpp     — joint config, angle ↔ step conversion, SO-101 defaults
  calibration.h/.cpp— end-stop sweep + NVS persistence (Preferences)
  kinematics.h/.cpp — FK computation and self-collision check (URDF geometry)
  monitor.h/.cpp    — FreeRTOS telemetry task, watchdog, SD blackbox
  can_bus.h/.cpp    — TWAI driver, Airbus-style voting, CAN ↔ WebSocket bridge
  main.cpp          — WiFi, WebSocket server, command dispatch, FK gate
docs/
  api.md            — full WebSocket API, CAN frames, calibration, wiring
  opensim-hil.md    — hardware-in-the-loop bridge setup and state mapping
```

---

## OpenSim HIL

The firmware integrates with [OpenSim](https://github.com/ghtomcat/opensim), a browser-based simulation engine. A Node.js bridge relays joint angles between the simulator and the ESP32 at 50 Hz. Because every `sync_move` still passes through the ESP32's FK and safety layer, a bad trajectory planned in the browser is rejected by the firmware — not by JavaScript. The 3-D renderer shows actual servo positions from telemetry, not commanded positions, so FK rejections are immediately visible as the sim and arm diverge.

```
Browser (Three.js)  ──STATE_PATCH──▶  hub.js  ──▶  robot_bridge.js (50 Hz)
                                                          │
                                                    ESP32 :80/ws
                                                          │
                                                  Feetech servo bus
                                                          │
                                         Telemetry ◀──────┘  back to sim
```

See [`docs/opensim-hil.md`](docs/opensim-hil.md) for setup instructions.

---

## Docs

| Document | Contents |
|----------|----------|
| [`docs/api.md`](docs/api.md) | Full WebSocket command reference, CAN message formats, voting flow, calibration, wiring |
| [`docs/opensim-hil.md`](docs/opensim-hil.md) | HIL bridge setup, state mapping, timing, safety guarantees |

---

## License

MIT — see [LICENSE](LICENSE).
