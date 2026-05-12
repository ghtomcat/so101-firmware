# SO-101 ESP32 Firmware

WiFi bridge firmware for the [Seeed Studio SO-101](https://www.seeedstudio.com/SO-ARM101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) robot arm, running on the [LILYGO T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485) ESP32 board.

Both a browser JS page and Python/LeRobot can control the arm over WebSocket simultaneously. The ESP32 enforces joint limits, monitors servo health, and logs everything to SD — independent of whatever controller is connected.

## Features

- **WebSocket API** — JSON commands for move, sync move, read, scan, stop, torque control
- **Angle-based control** — send degrees, not raw steps; calibration stored in NVS flash
- **Automated calibration** — end-stop sweep finds physical limits and computes joint center automatically
- **Safety layer** — joint limits clamped at firmware level, temperature/load/voltage thresholds, command watchdog cuts torque if controller goes silent
- **SD blackbox** — every telemetry frame and alert appended to `/so101.log` if a card is present
- **CAN bus** — multi-node support with Airbus-style N-of-M voting before executing moves; compile away with `CAN_ENABLE 0` for single-node builds
- **LeRobot compatible** — raw step escape hatch (`"pos"`) lets LeRobot talk directly; safety and logging apply to all controllers equally

## Hardware

| Board | [LILYGO T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485) |
|-------|--------------------------------------------------------------|
| Arm   | Seeed Studio SO-101 (Feetech STS3215 servos)                 |
| Bus   | RS-485 terminal A → servo S pin, terminal B → GND           |
| Power | External 7.4 V supply → servo V1 pins (not the T-CAN485)    |

The T-CAN485's onboard SP3485 RS-485 transceiver handles half-duplex switching automatically — no additional components needed.

## Quickstart

**1. Configure**

Edit `src/config.h`:
```c
#define WIFI_SSID  "your-network"
#define WIFI_PASS  "your-password"
#define CAN_ENABLE  0   // set to 1 for multi-node CAN setup
```

**2. Build and flash**
```bash
pio run -t upload
pio device monitor
```

The serial monitor prints the IP address once WiFi connects.

**3. Test from a browser console**
```js
const ws = new WebSocket("ws://<device-ip>/ws");
ws.onmessage = e => console.log(JSON.parse(e.data));

// Discover connected servos
ws.send(JSON.stringify({ cmd: "scan" }));

// Calibrate joint 1 (finds physical end-stops automatically)
ws.send(JSON.stringify({ cmd: "calibrate", id: 1 }));

// Move joint 1 to 45°
ws.send(JSON.stringify({ cmd: "move", id: 1, angle: 45.0 }));

// Emergency stop
ws.send(JSON.stringify({ cmd: "stop" }));
```

## Architecture

```
Browser (HTML/CSS/JS)  ──┐
                         ├── WebSocket ── ESP32 (T-CAN485) ── Feetech servo bus
Python / LeRobot       ──┘               │
                                         ├── CAN bus (optional, multi-node)
                                         ├── SD card (blackbox log)
                                         └── NVS flash (calibration)
```

The ESP32 is the safety boundary. Joint limits, torque cutoffs, watchdog, and the blackbox apply to every controller equally — including LeRobot policies. A policy that outputs an out-of-range angle is clamped silently; a controller that crashes causes torque to cut within 2 seconds.

The CAN layer borrows the Airbus EFCS architecture: multiple nodes must reach N-of-M quorum before a move executes. Any node in an alert state vetoes the network until the fault clears.

## Project structure

```
src/
  config.h          — WiFi, pins, safety thresholds, CAN settings
  feetech.h/.cpp    — Feetech STS3215 half-duplex UART driver
  joints.h/.cpp     — Joint config, angle ↔ step conversion, SO-101 defaults
  calibration.h/.cpp— End-stop sweep + NVS persistence (Preferences)
  monitor.h/.cpp    — FreeRTOS telemetry task, watchdog, SD blackbox
  can_bus.h/.cpp    — TWAI driver, voting, CAN ↔ WebSocket bridge
  main.cpp          — WiFi, WebSocket server, command dispatch
docs/
  api.md            — Full API reference (WebSocket commands + CAN frames)
```

## API

See [`docs/api.md`](docs/api.md) for the complete WebSocket command reference, CAN message formats, voting flow, calibration procedure, and wiring table.

## Calibration

Run once after assembly. The firmware sweeps each joint slowly to its physical end-stops, measures the load spike, and computes the center and safe range automatically:

```js
// Calibrate all joints one by one
for (let id = 1; id <= 6; id++) {
  ws.send(JSON.stringify({ cmd: "calibrate", id, speed: 150, margin_deg: 5 }));
}
```

Results are stored in ESP32 NVS flash and loaded on every boot. No reflashing needed.

## License

MIT — see [LICENSE](LICENSE).
