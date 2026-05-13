#pragma once

// --- WiFi ---
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASS "your-wifi-password"

// --- T-CAN485 RS-485 pins (from official Xinyuan-LilyGO/T-CAN485 config.h) ---
// The built-in SP3485 transceiver acts as the half-duplex buffer for the servo bus.
// Connect: RS-485 terminal A → servo data line, terminal B → GND.
#define PIN_5V_EN       16   // must be HIGH to power the RS-485 chip
#define SERVO_DIR_PIN   17   // RS485_EN_PIN: HIGH = transmit, LOW = receive
#define SERVO_TX_PIN    22   // RS485_TX_PIN
#define SERVO_RX_PIN    21   // RS485_RX_PIN
#define RS485_SE_PIN    19   // slew-rate: HIGH = full speed (required at 1 Mbps)

#define SERVO_BAUD      1000000

// --- Robot ---
#define SERVO_COUNT     6

// --- Safety thresholds ---
#define TEMP_LIMIT_C        70    // cut torque if any servo exceeds this (°C)
#define LOAD_LIMIT          800   // alert if load exceeds this (0–1000, 1000 = stall torque)
#define VOLT_MIN            60    // alert if bus voltage drops below this (tenths of V; 60 = 6.0 V)
#define WATCHDOG_MS         2000  // cut torque if no command received within this window (0 = disabled)

// --- Monitoring ---
#define TELEMETRY_MS        500   // WebSocket telemetry broadcast interval (ms)

// --- SD card (T-CAN485 onboard slot) ---
#define SD_MISO_PIN     2
#define SD_MOSI_PIN     15
#define SD_SCLK_PIN     14
#define SD_CS_PIN       13
#define SD_LOG_FILE     "/so101.log"   // log file path on SD card

// --- Forward-kinematics safety envelope ---
// Checked before every move command executes (WebSocket and CAN paths).
// Geometry is derived from the SO-101 URDF (so101_new_calib.urdf).
// Increase FK_MIN_HEIGHT_M if the arm is mounted above a surface it must not hit.
// Decrease FK_MAX_REACH_M to restrict the operational workspace further.
#define FK_MIN_HEIGHT_M     0.0f    // minimum z of any link node (meters above base plane)
#define FK_MAX_REACH_M      0.45f   // maximum TCP distance from base origin (meters)

// --- CAN bus (T-CAN485 onboard TJA1050 transceiver) ---
// Set CAN_ENABLE to 0 for single-node deployments where CAN is not used.
// All CAN code compiles away when CAN_ENABLE is 0.
#define CAN_ENABLE          1

#if CAN_ENABLE
#define CAN_TX_PIN          26      // from T-CAN485 schematic
#define CAN_RX_PIN          27
#define CAN_SE_PIN          23      // TJA1050 standby: LOW = active, HIGH = standby
#define CAN_NODE_ID         1       // this node's ID on the bus (1–15)
#define CAN_BAUD_KBPS       500     // 500 Kbps — standard for short industrial runs

// Voting (Airbus-style N-of-M command consensus).
// Set CAN_VOTE_QUORUM to 1 to approve all commands from this node alone
// (effectively disables voting while keeping the infrastructure ready).
#define CAN_VOTE_QUORUM     2       // approvals required before executing a voted move
#define CAN_VOTE_TIMEOUT_MS 150     // max wait for votes (ms)

#define CAN_HEARTBEAT_MS    1000    // heartbeat broadcast interval (ms)
#endif
