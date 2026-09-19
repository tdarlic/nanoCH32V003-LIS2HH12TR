#!/usr/bin/env python3
"""Read LIS2HH12 accelerometer data streamed by the nanoCH32V003 firmware
over the WCH-LinkE's UART bridge. Minimal non-interactive alternative to
dashboard.py.

Usage:
    python3 read_accel.py [port]              # stream readings
    python3 read_accel.py [port] --stream=off  # pause streaming on the MCU, then exit
    python3 read_accel.py [port] --stream=on   # resume streaming, then exit
"""
import sys
import time

from lis2hh12_link import Link, parse_acc

DEFAULT_PORT = "/dev/ttyACM0"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else DEFAULT_PORT
    stream_arg = next((a for a in sys.argv[1:] if a.startswith("--stream=")), None)

    link = Link(port)
    try:
        if stream_arg:
            state = stream_arg.split("=", 1)[1].upper()
            if state not in ("ON", "OFF"):
                print(f"invalid --stream value: {state!r} (must be on/off)")
                return
            link.send(f"STREAM {state}")
            lines = link.wait_for(("OK", "ERR"), timeout=1.0)
            print(*lines, sep="\n")
            return

        while True:
            for line in link.poll_lines():
                acc = parse_acc(line)
                if acc:
                    x, y, z, temp_mc = acc
                    mag = (x * x + y * y + z * z) ** 0.5
                    print(f"x={x:6d} mg  y={y:6d} mg  z={z:6d} mg  "
                          f"|g|={mag / 1000:.3f} g  temp={temp_mc / 1000:.1f} C")
                else:
                    print(f"[mcu] {line}")
            time.sleep(0.02)
    finally:
        link.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
