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
import select
import socket
import struct
import sys
import time


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


def _ws_recv_cmd(sock: socket.socket) -> dict:
    """Receive frames, skipping broadcast telemetry/alerts, until a command response arrives."""
    while True:
        msg = json.loads(ws_recv(sock))
        if "type" not in msg:
            return msg


def jog(ip: str, port: int) -> None:
    """Interactive joint jog: select a servo then use arrow keys to move it."""
    try:
        import tty
        import termios
    except ImportError:
        sys.exit("--jog requires a Unix terminal (not available on Windows)")

    try:
        sock = ws_connect(ip, port)
    except (OSError, ConnectionError) as e:
        sys.exit(f"Connection failed: {e}")

    try:
        json.loads(ws_recv(sock))  # welcome frame

        # Scan for live servos
        ws_send(sock, json.dumps({"cmd": "scan"}))
        resp = _ws_recv_cmd(sock)
        found = resp.get("found", [])
        if not found:
            print("No servos found. Check wiring and power.")
            return

        print(f"Available servos: {found}")
        try:
            sid = int(input("Servo ID to jog: "))
        except (ValueError, EOFError):
            return
        if sid not in found:
            print(f"ID {sid} not found.")
            return

        # Read initial angle
        ws_send(sock, json.dumps({"cmd": "read", "id": sid}))
        st = _ws_recv_cmd(sock)
        if not st.get("ok"):
            print(f"Cannot read servo {sid}.")
            return

        idx   = found.index(sid)
        angle = float(st["angle"])
        load  = st.get("load", 0)
        step  = 5.0

        print(f"\nServo {sid}   ←→ move   ↑↓ step   n/p next/prev servo   q quit\n")

        fd           = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        def show(note: str = "") -> None:
            suffix = f"  {note}" if note else ""
            print(f"\r  [{sid}]  angle={angle:7.1f}°   step={step:4.1f}°   load={load & 0x7FFF:4d}{suffix}          ",
                  end="", flush=True)

        def lock_others(active_sid: int) -> None:
            """Hold all servos except active_sid at their current position."""
            sock.settimeout(2.0)
            try:
                for other_id in found:
                    if other_id == active_sid:
                        continue
                    ws_send(sock, json.dumps({"cmd": "read", "id": other_id}))
                    try:
                        r = _ws_recv_cmd(sock)
                        if r.get("ok"):
                            ws_send(sock, json.dumps({
                                "cmd": "move", "id": other_id,
                                "angle": float(r["angle"]), "speed": 200, "acc": 10,
                                "no_fk": True,
                            }))
                            _ws_recv_cmd(sock)  # consume ACK
                    except (socket.timeout, json.JSONDecodeError, KeyError):
                        pass
            finally:
                sock.settimeout(None)

        def switch_servo(new_idx: int) -> tuple:
            """Switch to a different servo; returns (new_sid, new_angle, new_idx, new_load)."""
            new_sid = found[new_idx]
            ws_send(sock, json.dumps({"cmd": "read", "id": new_sid}))
            sock.settimeout(2.0)
            try:
                r = _ws_recv_cmd(sock)
                new_angle = float(r["angle"]) if r.get("ok") else 0.0
                new_load  = r.get("load", 0)  if r.get("ok") else 0
            except socket.timeout:
                new_angle = 0.0
                new_load  = 0
            finally:
                sock.settimeout(None)
            lock_others(new_sid)
            return new_sid, new_angle, new_idx, new_load

        lock_others(sid)
        show()

        try:
            tty.setraw(fd)
            buf = b""
            while True:
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if ready:
                    buf += os.read(fd, 32)  # bypass Python buffering

                if not buf:
                    continue

                # Need 3 bytes for an escape sequence; wait for more if incomplete.
                if buf[0:1] == b"\x1b" and len(buf) < 3:
                    continue

                # Parse one key event from the front of buf.
                if buf[0:1] in (b"q", b"\x03", b"\x04"):
                    break

                elif buf[0:1] == b"n":          # next servo
                    buf = buf[1:]
                    sid, angle, idx, load = switch_servo((idx + 1) % len(found))
                    show()
                    continue

                elif buf[0:1] == b"p":          # previous servo
                    buf = buf[1:]
                    sid, angle, idx, load = switch_servo((idx - 1) % len(found))
                    show()
                    continue

                elif buf[0:3] == b"\x1b[C":     # →
                    delta, buf = +step, buf[3:]
                elif buf[0:3] == b"\x1b[D":     # ←
                    delta, buf = -step, buf[3:]
                elif buf[0:3] == b"\x1b[A":     # ↑  larger step
                    step = min(45.0, step + (5.0 if step >= 5.0 else 1.0))
                    buf = buf[3:]
                    show()
                    continue
                elif buf[0:3] == b"\x1b[B":     # ↓  smaller step
                    step = max(1.0, step - (5.0 if step > 5.0 else 1.0))
                    buf = buf[3:]
                    show()
                    continue
                else:
                    buf = buf[1:]
                    continue

                new_angle = round(angle + delta, 1)
                ws_send(sock, json.dumps({
                    "cmd": "move", "id": sid,
                    "angle": new_angle, "speed": 300, "acc": 30,
                    "no_fk": True,
                }))

                sock.settimeout(1.0)
                try:
                    ack = _ws_recv_cmd(sock)
                    if ack.get("ok"):
                        angle = new_angle
                        # Refresh load from a quick read after each move.
                        ws_send(sock, json.dumps({"cmd": "read", "id": sid}))
                        try:
                            st = _ws_recv_cmd(sock)
                            if st.get("ok"):
                                load = st.get("load", load)
                        except socket.timeout:
                            pass
                        show()
                    else:
                        reason = ack.get("fk") or ack.get("err", "rejected")
                        show(f"[{reason}]")
                        time.sleep(0.4)
                        show()
                except socket.timeout:
                    angle = new_angle
                    show()
                finally:
                    sock.settimeout(None)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print()

    except (ConnectionError, json.JSONDecodeError) as e:
        print(f"\nError: {e}")
    finally:
        sock.close()


def cart_jog(ip: str, port: int) -> None:
    """Cartesian TCP jog: WASD=X/Y  R/F=Z  arrows=step  q=quit."""
    try:
        import tty
        import termios
    except ImportError:
        sys.exit("--cart-jog requires a Unix terminal (not available on Windows)")

    try:
        sock = ws_connect(ip, port)
    except (OSError, ConnectionError) as e:
        sys.exit(f"Connection failed: {e}")

    try:
        json.loads(ws_recv(sock))  # welcome frame

        step = 0.010  # metres

        # Query current TCP from firmware before entering jog loop.
        ws_send(sock, json.dumps({"cmd": "cart_jog", "dx": 0.0, "dy": 0.0, "dz": 0.0}))
        try:
            sock.settimeout(2.0)
            r = _ws_recv_cmd(sock)
            tcp = r.get("tcp", {"x": 0.0, "y": 0.0, "z": 0.0})
        except socket.timeout:
            tcp = {"x": 0.0, "y": 0.0, "z": 0.0}
        finally:
            sock.settimeout(None)

        print("\nCartesian jog:  W/S=+X/-X   A/D=+Y/-Y   R/F=+Z/-Z   ↑↓=step   q=quit\n")

        fd           = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        def show(note: str = "") -> None:
            suffix = f"  {note}" if note else ""
            print(
                f"\r  tcp=({tcp['x']:+.3f}, {tcp['y']:+.3f}, {tcp['z']:+.3f})"
                f"  step={step*1000:.0f}mm{suffix}          ",
                end="", flush=True)

        show()

        try:
            tty.setraw(fd)
            buf = b""
            while True:
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if ready:
                    buf += os.read(fd, 32)

                if not buf:
                    continue
                if buf[0:1] == b"\x1b" and len(buf) < 3:
                    continue

                if buf[0:1] in (b"q", b"\x03", b"\x04"):
                    break
                elif buf[0:3] == b"\x1b[A":     # ↑ larger step
                    step = min(0.050, round(step * 2, 3))
                    buf  = buf[3:]
                    show()
                    continue
                elif buf[0:3] == b"\x1b[B":     # ↓ smaller step
                    step = max(0.001, round(step / 2, 3))
                    buf  = buf[3:]
                    show()
                    continue

                key = buf[0:1]
                buf = buf[1:]

                delta = {}
                if   key == b"w": delta = {"dx": +step, "dy": 0.0, "dz": 0.0}
                elif key == b"s": delta = {"dx": -step, "dy": 0.0, "dz": 0.0}
                elif key == b"a": delta = {"dx": 0.0, "dy": +step, "dz": 0.0}
                elif key == b"d": delta = {"dx": 0.0, "dy": -step, "dz": 0.0}
                elif key == b"r": delta = {"dx": 0.0, "dy": 0.0, "dz": +step}
                elif key == b"f": delta = {"dx": 0.0, "dy": 0.0, "dz": -step}
                else:
                    continue

                ws_send(sock, json.dumps({"cmd": "cart_jog", **delta, "speed": 300, "acc": 30}))
                sock.settimeout(1.0)
                try:
                    ack = _ws_recv_cmd(sock)
                    if ack.get("ok") or "tcp" in ack:
                        tcp = ack.get("tcp", tcp)
                        note = " [~]" if not ack.get("ok") else ""
                        show(note)
                    else:
                        reason = ack.get("fk") or ack.get("err", "rejected")
                        show(f"[{reason}]")
                        time.sleep(0.3)
                        show()
                except socket.timeout:
                    show("[timeout]")
                finally:
                    sock.settimeout(None)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print()

    except (ConnectionError, json.JSONDecodeError) as e:
        print(f"\nError: {e}")
    finally:
        sock.close()


def switch_id(ip: str, port: int, old_id: int, new_id: int) -> None:
    try:
        sock = ws_connect(ip, port)
    except (OSError, ConnectionError) as e:
        sys.exit(f"Connection failed: {e}")

    try:
        json.loads(ws_recv(sock))  # welcome frame

        ws_send(sock, json.dumps({"cmd": "scan"}))
        found = set(_ws_recv_cmd(sock).get("found", []))

        if old_id not in found:
            print(f"Servo {old_id} not found on bus.")
            return

        if new_id in found:
            temp = next(i for i in range(10, 254) if i not in found)
            steps = [(old_id, temp), (new_id, old_id), (temp, new_id)]
            print(f"Both IDs present — swapping via temp {temp}: "
                  f"{old_id}→{temp}, {new_id}→{old_id}, {temp}→{new_id}")
        else:
            steps = [(old_id, new_id)]
            print(f"Renaming servo {old_id} → {new_id}")

        for src, dst in steps:
            print(f"  {src} → {dst} …", end=" ", flush=True)
            ws_send(sock, json.dumps({"cmd": "switch_id", "old_id": src, "new_id": dst}))
            r = _ws_recv_cmd(sock)
            if r.get("ok"):
                print("OK")
            else:
                print(f"FAILED: {r.get('err', r)}")
                return

        print("\nDone. NVS calibration is now invalid for these servos; recalibrate when all IDs are correct.")

    except (ConnectionError, json.JSONDecodeError) as e:
        sys.exit(f"Error: {e}")
    finally:
        sock.close()


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
    parser.add_argument("--jog", action="store_true",
                        help="Interactive joint jog: select a servo and move it with arrow keys")
    parser.add_argument("--cart-jog", action="store_true",
                        help="Cartesian TCP jog: WASD=X/Y  R/F=Z  arrows=step")
    parser.add_argument("--switch-id", type=int, nargs=2, metavar=("OLD", "NEW"),
                        help="Change a servo's EEPROM ID  e.g. --switch-id 4 6")
    args = parser.parse_args()

    if args.calibrate:
        calibrate(args.ip, args.port, args.calibrate)
    elif args.jog:
        jog(args.ip, args.port)
    elif args.cart_jog:
        cart_jog(args.ip, args.port)
    elif args.switch_id:
        switch_id(args.ip, args.port, args.switch_id[0], args.switch_id[1])
    else:
        discover(args.ip, args.port, debug=args.debug)


if __name__ == "__main__":
    main()
