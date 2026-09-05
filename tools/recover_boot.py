#!/usr/bin/env python3
"""Recover the BK7238 bootloader region (0x0..0x11000) reliably.

The T1-2S-NL bootloader isn't in bk7231tools' DB ("Unknown bootloader CRC"),
so the automatic boot-protection bypass is unreliable for the protected boot
sectors. This forces the SR fully unprotected, writes with the bypass toggled
explicitly, and verifies by reading back through the SAME path — retrying the
whole sequence until the first 256 bytes match.
"""
import sys
import time

import serial
from serial.tools import list_ports
from bk7231tools.serial import BK7231Serial

MY_SERIAL = "5C94136993"
BRIDGE_EXIT = bytes([0xDE, 0xAD, 0xBE, 0xEF]) + b"BKJIG-EXIT42"
BOOT = open(sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\rose\AppData\Local\Temp\stock_boot.bin", "rb").read()


def find_port():
    for p in list_ports.comports():
        if p.serial_number and MY_SERIAL in p.serial_number:
            return p.device
    raise SystemExit("jig not found")


def enter(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b"\nFLASHMODE\n")
    t0 = time.time()
    buf = b""
    while time.time() - t0 < 3:
        buf += s.read(256)
        if b"ENTERING BRIDGE" in buf:
            break
    s.close()


def leave(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    s.write(BRIDGE_EXIT)
    s.flush()
    time.sleep(1.0)
    s.close()


def try_once(port, force_bypass):
    enter(port)
    bk = None
    try:
        bk = BK7231Serial(port, baudrate=115200, link_timeout=25.0, cmnd_timeout=3.0)
        bk.info = lambda *a: None
        bk.connect()
        # Force the bypass state we want to test.
        bk.boot_protection_bypass = force_bypass
        bk.flash_unprotect()
        # Write the whole boot region in 4K sectors.
        for off in range(0, len(BOOT), 4096):
            chunk = BOOT[off:off + 4096]
            bk.flash_write_4k(off, chunk, crc_check=False)
        # Verify first + a middle + last sector.
        ok = True
        for off in (0x0, 0x8000, 0x10000):
            rb = bk.flash_read_bytes(off, 256, crc_check=False)
            if rb != BOOT[off:off + 256]:
                ok = False
                break
        return ok
    except Exception as e:
        print(f"  attempt error: {e}")
        return False
    finally:
        if bk:
            try:
                bk.reboot_chip()
            except Exception:
                pass
            try:
                bk.close()
            except Exception:
                pass
        leave(port)


def main():
    port = find_port()
    print(f"boot image: {len(BOOT)} bytes")
    for bypass in (True, False):
        for attempt in range(3):
            print(f"bypass={bypass} attempt={attempt}...")
            if try_once(port, bypass):
                print(f"SUCCESS with boot_protection_bypass={bypass}")
                return
    print("FAILED: bootloader region still not writable by any method tried")
    sys.exit(1)


if __name__ == "__main__":
    main()
