#!/usr/bin/env python3
"""
Discover Feetech servos on an SO-101 arm via the ESP32 WebSocket.

Usage:
    python tools/discover.py 192.168.1.100

Requirements:
    pip install websockets
"""

import argparse
import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    sys.exit("Missing dependency — run: pip install websockets")


async def discover(ip: str, port: int = 80) -> None:
    url = f"ws://{ip}:{port}/ws"
    print(f"Connecting to {url} ...")

    try:
        async with websockets.connect(url, open_timeout=5) as ws:
            # Welcome frame sent by firmware on connect
            welcome = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
            sd = "SD logging ON" if welcome.get("sd_log") else "SD logging OFF"
            print(f"Connected  joints={welcome.get('joints', '?')}  {sd}\n")

            # Scan the servo bus
            await ws.send(json.dumps({"cmd": "scan"}))
            resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))

            if not resp.get("ok"):
                print(f"Scan failed: {resp.get('err', resp)}")
                return

            found = resp.get("found", [])
            if not found:
                print("No servos found. Check wiring and power.")
                return

            print(f"Found {len(found)} servo(s): {found}\n")

            # Read full state of each servo
            print(f"{'ID':<4}  {'Angle':>8}  {'Pos':>5}  {'Load':>5}  {'Temp':>5}  {'Volt':>5}")
            print("─" * 44)
            for sid in found:
                await ws.send(json.dumps({"cmd": "read", "id": sid}))
                r = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
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

    except (OSError, TimeoutError, asyncio.TimeoutError) as e:
        sys.exit(f"Connection failed: {e}")
    except websockets.exceptions.WebSocketException as e:
        sys.exit(f"WebSocket error: {e}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Discover SO-101 servos via ESP32 WebSocket",
        epilog="The IP address is printed to the serial monitor on boot.",
    )
    parser.add_argument("ip", help="ESP32 IP address  e.g. 192.168.1.100")
    parser.add_argument("--port", type=int, default=80, metavar="PORT",
                        help="WebSocket port (default: 80)")
    args = parser.parse_args()

    asyncio.run(discover(args.ip, args.port))


if __name__ == "__main__":
    main()
