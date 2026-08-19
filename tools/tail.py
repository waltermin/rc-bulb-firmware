#!/usr/bin/env python3
"""Persistently tail an ESP8266 serial console WITHOUT resetting the board.

Opening a COM port normally toggles DTR/RTS, which trips the auto-reset circuit.
Holding both deasserted from the moment of open avoids that, so this attaches to
a running board without rebooting it. Everything read is echoed to stdout AND
appended to a logfile, so a second viewer can follow with `tail -f <logfile>`
(only one process may hold the COM port, but any number can read the file).

    python3 tools/tail.py [PORT] [BAUD] [LOGFILE]
    python3 tools/tail.py COM8 74880 dfu-test.log
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial required: pip install pyserial")

port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 74880
logpath = sys.argv[3] if len(sys.argv) > 3 else "dfu-test.log"

s = serial.Serial()
s.port = port
s.baudrate = baud
# Key part: keep DTR and RTS deasserted so opening the port does not reset the
# board. (pyserial applies these stored states when open() runs.)
s.dtr = False
s.rts = False
s.timeout = 0.2
s.open()

banner = f"\n---- tail attached to {port} @ {baud} ({time.strftime('%H:%M:%S')}) ----\n"
sys.stdout.write(banner)
sys.stdout.flush()

with open(logpath, "a", buffering=1, encoding="utf-8", errors="replace") as f:
    f.write(banner)
    try:
        while True:
            data = s.read(4096)
            if data:
                text = data.decode("utf-8", "replace")
                f.write(text)
                # ascii-safe for the Windows console
                sys.stdout.write(text.encode("ascii", "replace").decode("ascii"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
