#!/usr/bin/env python3
"""Flash the T1-2S-NL through the BKJIG bridge (ESP32-S3 UART jig).

Puts the jig into FLASHMODE (transparent bridge + automatic CEN pulses that
catch the BK7238 BootROM), runs `ltchiptool flash write` over the bridge at a
fixed 115200, then exits the bridge and reboots the BK into the new app.

Usage:
  python tools/flash_via_jig.py [file]   (default .pio/build/t1-2s/firmware.uf2)
"""
import subprocess
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

JIG_USB_SERIAL = "5C94136993"   # Claude's ESP32-S3; do not touch other CH343s
BRIDGE_EXIT = bytes([0xDE, 0xAD, 0xBE, 0xEF]) + b"BKJIG-EXIT42"
DEFAULT_IMAGE = Path(__file__).parent.parent / ".pio" / "build" / "t1-2s" / "firmware.uf2"


def find_port():
    for p in list_ports.comports():
        if p.serial_number and JIG_USB_SERIAL in p.serial_number:
            return p.device
    raise SystemExit(f"ERROR: jig board (USB serial {JIG_USB_SERIAL}) not found")


def jig_cmd(port, cmd, expect, timeout=3):
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b"\n" + cmd.encode() + b"\n")
    buf = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        buf += s.read(256)
        if expect.encode() in buf:
            s.close()
            return True
    s.close()
    raise SystemExit(f"ERROR: jig did not answer {cmd!r} (got {buf!r})")


def exit_bridge_and_reboot(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    s.write(BRIDGE_EXIT)
    s.flush()
    time.sleep(1.5)  # ESP restarts to command mode
    s.reset_input_buffer()
    s.write(b"\nREBOOT\n")
    time.sleep(0.5)
    s.close()


def main():
    image = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_IMAGE
    if not image.exists():
        raise SystemExit(f"ERROR: image not found: {image}")
    port = find_port()
    print(f"jig on {port}; image {image.name} ({image.stat().st_size} bytes)")

    jig_cmd(port, "FLASHMODE", "ENTERING BRIDGE")
    print("bridge mode entered, running ltchiptool...")
    try:
        subprocess.run(
            [sys.executable, "-m", "ltchiptool", "flash", "write",
             str(image), "-d", port, "-b", "115200"],
            check=True,
        )
    finally:
        exit_bridge_and_reboot(port)
        print("bridge exited, BK rebooted into app")


if __name__ == "__main__":
    main()
