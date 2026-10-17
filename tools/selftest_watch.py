#!/usr/bin/env python3
"""Watch a LEMTONE_SELFTEST build run on a bulb's serial console.

Attaches WITHOUT resetting (DTR/RTS held deasserted, like tail.py), echoes everything
and tees it to a logfile, and exits when the self-test prints its final verdict:
0 on "SELFTEST PASS", 1 on "SELFTEST FAIL", 2 on timeout.

    python tools/selftest_watch.py <PORT> [--log selftest.log] [--timeout 300]

Build the image with:  BULB_SELFTEST=1 ./build.sh bulb  (or $env:BULB_SELFTEST=1; .\build.ps1 -Device bulb)
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial required: pip install pyserial")

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("port", help="bulb console serial port (required; no default)")
ap.add_argument("--baud", type=int, default=74880)
ap.add_argument("--log", default="selftest.log")
ap.add_argument("--timeout", type=float, default=300, help="seconds to wait for the verdict")
a = ap.parse_args()

s = serial.Serial()
s.port, s.baudrate, s.timeout = a.port, a.baud, 0.2
s.dtr = s.rts = False   # do not reset the board on open
s.open()

code, buf, t_end = 2, "", time.time() + a.timeout
with open(a.log, "a", buffering=1, encoding="utf-8", errors="replace") as f:
    f.write(f"\n---- selftest_watch on {a.port} @ {a.baud} ({time.strftime('%H:%M:%S')}) ----\n")
    try:
        while time.time() < t_end:
            data = s.read(4096)
            if not data:
                continue
            text = data.decode("utf-8", "replace")
            f.write(text)
            sys.stdout.write(text.encode("ascii", "replace").decode("ascii"))
            sys.stdout.flush()
            buf = (buf + text)[-512:]
            if "SELFTEST PASS" in buf or "SELFTEST FAIL" in buf:
                code = 0 if "SELFTEST PASS" in buf else 1
                t_end = min(t_end, time.time() + 1.0)   # catch the trailing line
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
print({0: "\n== selftest passed", 1: "\n== selftest FAILED", 2: "\n== no verdict before timeout"}[code])
sys.exit(code)
