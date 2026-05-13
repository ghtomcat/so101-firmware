#!/usr/bin/env python3
"""
Scan Feetech servos directly via URT-2 USB debugger.
Tries common baud rates and reports which servos respond.

Usage:
    python3 tools/urt_scan.py /dev/cu.usbmodem5B790332241
"""

import sys
import time
import serial   # pip install pyserial


BAUD_RATES = [1_000_000, 500_000, 250_000, 115_200, 57_600]


def make_ping(servo_id: int) -> bytes:
    chk = (~(servo_id + 2 + 1)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, 0x02, 0x01, chk])


def ping(ser: serial.Serial, servo_id: int) -> bytes | None:
    pkt = make_ping(servo_id)
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    time.sleep(0.05)               # 50 ms — plenty of time for response
    raw = ser.read(32)
    if not raw:
        return None
    # URT-2 echoes TX bytes back on RX — strip them if present
    if raw[: len(pkt)] == pkt:
        raw = raw[len(pkt):]
    return raw if raw else None


def scan_at_baud(port: str, baud: int) -> None:
    print(f"\n── {baud:,} bps ─────────────────────────────")
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            found = False
            for sid in range(1, 8):          # 1–7 covers all SO-101 joints + spare
                resp = ping(ser, sid)
                if resp:
                    found = True
                    print(f"  ID {sid}: {resp.hex(' ')}  ", end="")
                    # Validate: FF FF ID 02 ERR CHK
                    if (len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF
                            and resp[2] == sid and resp[3] == 0x02):
                        err = resp[4]
                        print(f"{'OK' if err == 0 else f'ERR=0x{err:02X}'}")
                    else:
                        print("(unexpected format)")
                else:
                    print(f"  ID {sid}: —")
            if not found:
                print("  No response at this baud rate.")
    except serial.SerialException as e:
        print(f"  Serial error: {e}")


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5B790332241"
    print(f"Scanning {port} ...")
    for baud in BAUD_RATES:
        scan_at_baud(port, baud)
    print("\nDone.")


if __name__ == "__main__":
    main()
