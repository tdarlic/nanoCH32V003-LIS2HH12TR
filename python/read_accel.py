#!/usr/bin/env python3
"""Read LIS2HH12 accelerometer data streamed by the nanoCH32V003 firmware
over the WCH-LinkE's UART bridge.

Usage:
    python3 read_accel.py [port]        # stream readings
    python3 read_accel.py [port] --pause-toggle   # send 'p' once, then exit
"""
import sys
import serial

DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else DEFAULT_PORT
    toggle_only = "--pause-toggle" in sys.argv

    with serial.Serial(port, BAUD, timeout=1) as ser:
        if toggle_only:
            ser.write(b"p")
            return

        for raw in ser:
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("ACC,"):
                x, y, z = (int(v) for v in line[4:].split(","))
                mag = (x * x + y * y + z * z) ** 0.5
                print(f"x={x:6d} mg  y={y:6d} mg  z={z:6d} mg  |g|={mag / 1000:.3f} g")
            else:
                print(f"[mcu] {line}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
