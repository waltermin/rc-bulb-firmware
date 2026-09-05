#!/usr/bin/env python3
"""Single-session: open jig, pulse CEN (reset BK) while listening, stream UART."""
import sys, time
import serial
from serial.tools import list_ports

MY_SERIAL = "5C94136993"


def find_port():
    for p in list_ports.comports():
        if p.serial_number and MY_SERIAL in p.serial_number:
            return p.device
    raise SystemExit("jig not found")


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15
    s = serial.Serial(find_port(), 115200, timeout=0.1)
    time.sleep(2.0)  # let BKJIG boot after the DTR reset from opening the port
    s.reset_input_buffer()
    # enter MON on the jig, then pulse CEN to reset the BK while we're listening
    s.write(b"\nCEN PULSE 100\n")
    time.sleep(0.15)
    s.write(b"MON\n")
    t0 = time.time()
    while time.time() - t0 < secs:
        chunk = s.read(4096)
        if chunk:
            sys.stdout.buffer.write(chunk)
            sys.stdout.flush()
    s.write(b"x\n")
    s.close()


if __name__ == "__main__":
    main()
