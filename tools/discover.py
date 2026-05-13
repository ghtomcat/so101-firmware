#!/usr/bin/env python3
"""
Discover and calibrate Feetech servos on an SO-101 arm via the ESP32 WebSocket.
No external dependencies — Python stdlib only.

Usage:
    python3 tools/discover.py 192.168.1.100
    python3 tools/discover.py 192.168.1.100 --debug
    python3 tools/discover.py 192.168.1.100 --calibrate 4
    python3 tools/discover.py 192.168.1.100 --calibrate 4 5 6
"""

import argparse
import base64
import json
import os
import socket
import struct
import sys


# ── Minimal WebSocket client (RFC 6455) ─────────────────────────────────────

def _recv_exactly(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed by server")
        buf += chunk
    return bytes(buf)


def ws_connect(host: str, port: int = 80, path: str = "/ws") -> socket.socket:
    sock = socket.create_connection((host, port), timeout=5)
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n"
        f"\r\n"
    ).encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        resp += sock.recv(4096)
    if b" 101 " not in resp.split(b"\r\n")[0]:
        raise ConnectionError(f"WebSocket upgrade failed: {resp[:200]!r}")
    return sock


def ws_send(sock: socket.socket, text: str) -> None:
    data = text.encode()
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    n = len(data)
    if n < 126:
        header = struct.pack("BB", 0x81, 0x80 | n)
    elif n < 65536:
        header = struct.pack("!BBH", 0x81, 0xFE, n)
    else:
        header = struct.pack("!BBQ", 0x81, 0xFF, n)
    sock.sendall(header + mask + masked)


def ws_recv(sock: socket.socket) -> str:
    while True:
        hdr = _recv_exactly(sock, 2)
        opcode = hdr[0] & 0x0F
        is_masked = bool(hdr[1] & 0x80)
        length = hdr[1] & 0x7F
        if length == 126:
            length = struct.unpack("!H", _recv_exactly(sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", _recv_exactly(sock, 8))[0]
        mask_key = _recv_exactly(sock, 4) if is_masked else b""
        payload = _recv_exactly(sock, length)
        if is_masked:
            payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
        if opcode == 0x8:                           # close
            raise ConnectionError("Server closed the WebSocket")
        if opcode == 0x9:                           # ping → pong
            sock.sendall(struct.pack("BB", 0x8A, len(payload)) + payload)
            continue
        if opcode in (0x1, 0x2):                    # text / binary
            return payload.decode()


# ── Discovery ────────────────────────────────────────────────────────────────

def discover(ip: str, port: int = 80, debug: bool = False) -> None:
    url = f"ws://{ip}:{port}/ws"
    print(f"Connecting to {url} ...")

    try:
        sock = ws_connect(ip, port)
    except (OSError, ConnectionError) as e:
        sys.exit(f"Connection failed: {e}")

    try:
        welcome = json.loads(ws_recv(sock))
        sd = "SD logging ON" if welcome.get("sd_log") else "SD logging OFF"
        print(f"Connected  joints={welcome.get('joints', '?')}  {sd}\n")

        if debug:
            _debug_scan(sock)
            return

        ws_send(sock, json.dumps({"cmd": "scan"}))
        resp = json.loads(ws_recv(sock))

        if not resp.get("ok"):
            print(f"Scan failed: {resp.get('err', resp)}")
            return

        found = resp.get("found", [])
        if not found:
            print("No servos found. Check wiring and power.")
            return

        print(f"Found {len(found)} servo(s): {found}\n")

        print(f"{'ID':<4}  {'Angle':>8}  {'Pos':>5}  {'Load':>5}  {'Temp':>5}  {'Volt':>5}")
        print("─" * 44)
        for sid in found:
            ws_send(sock, json.dumps({"cmd": "read", "id": sid}))
            r = json.loads(ws_recv(sock))
            if r.get("ok"):
                print(
                    f"{r['id']:<4}  "
                    f"{float(r['angle']):>7.1f}°  "
                    f"{r['pos']:>5}  "
                    f"{r['load']:>5}  "
                    f"{r['temp']:>4}°C  "
                    f"{r['volt'] / 10:>4.1f}V"
                )
            else:
                print(f"{sid:<4}  read error: {r.get('err', '?')}")

    except (ConnectionError, json.JSONDecodeError) as e:
        sys.exit(f"Error: {e}")
    finally:
        sock.close()


def calibrate(ip: str, port: int, ids: list[int]) -> None:
    url = f"ws://{ip}:{port}/ws"
    print(f"Connecting to {url} ...")

    try:
        sock = ws_connect(ip, port)
    except (OSError, ConnectionError) as e:
        sys.exit(f"Connection failed: {e}")

    try:
        welcome = json.loads(ws_recv(sock))
        print(f"Connected  joints={welcome.get('joints', '?')}\n")
        print("Make sure the arm can move freely through its full range.\n")

        any_ok = False
        for sid in ids:
            print(f"Calibrating servo {sid} — sweeping to end stops …", flush=True)
            ws_send(sock, json.dumps({
                "cmd":         "calibrate",
                "id":          sid,
                "speed":       400,
                "margin_deg":  5.0,
                "load_thresh": 400,
                "timeout_ms":  15000,
            }))

            sock.settimeout(40)
            while True:
                r = json.loads(ws_recv(sock))
                if "type" not in r:  # skip telemetry + alert broadcasts
                    break
            sock.settimeout(None)

            if not r.get("ok"):
                print(f"  ID {sid}: FAILED — {r.get('err', r)}")
            else:
                any_ok = True
                print(
                    f"  ID {sid}: OK  center={r['center']}  "
                    f"range={r['range_deg']}°  "
                    f"[{r['min_deg']}° … {r['max_deg']}°]"
                )

        if any_ok:
            print("\nCalibration saved to NVS.")
        else:
            print("\nNo servos calibrated successfully.")

    except (ConnectionError, json.JSONDecodeError) as e:
        sys.exit(f"Error: {e}")
    finally:
        sock.close()


def _debug_scan(sock) -> None:
    print("Running bus diagnostic (debug_scan) …\n")
    ws_send(sock, json.dumps({"cmd": "debug_scan"}))
    resp = json.loads(ws_recv(sock))

    if not resp.get("ok"):
        print(f"debug_scan failed: {resp}")
        return

    print(f"{'ID':<4}  {'RX bytes':>8}  {'Valid':>6}  Raw hex")
    print("─" * 60)
    for sv in resp.get("servos", []):
        sid    = sv["id"]
        n      = sv["rx"]
        valid  = sv["valid"]
        hexstr = sv.get("hex", "")
        status = "OK" if valid else ("TIMEOUT" if n == 0 else "BAD CHK")
        print(f"{sid:<4}  {n:>8}  {status:>6}  {hexstr}")

    print()
    any_rx  = any(sv["rx"] > 0 for sv in resp.get("servos", []))
    any_ok  = any(sv["valid"]  for sv in resp.get("servos", []))

    if any_ok:
        print("Servo(s) responding correctly — run without --debug to see full state.")
    elif any_rx:
        print("Bytes received but checksum failed.")
        print("  → Check baud rate (should be 1 000 000), or possible noise on the bus.")
    else:
        print("Nothing received from any servo.")
        print("  → Check: RS-485 terminal B connected to GND?")
        print("  → Check: servo power ≥ 6 V?")
        print("  → Check: DIR pin (GPIO 17) not shorted?")
        print("  → Check: A/B terminals not swapped?")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Discover SO-101 servos via ESP32 WebSocket",
        epilog="The IP address is printed to the serial monitor on boot.",
    )
    parser.add_argument("ip", help="ESP32 IP address  e.g. 192.168.1.100")
    parser.add_argument("--port", type=int, default=80, metavar="PORT",
                        help="WebSocket port (default: 80)")
    parser.add_argument("--debug", action="store_true",
                        help="Raw bus diagnostic: show RX bytes per servo ping")
    parser.add_argument("--calibrate", type=int, nargs="+", metavar="ID",
                        help="Calibrate one or more servos by ID  e.g. --calibrate 4 5 6")
    args = parser.parse_args()

    if args.calibrate:
        calibrate(args.ip, args.port, args.calibrate)
    else:
        discover(args.ip, args.port, debug=args.debug)


if __name__ == "__main__":
    main()
