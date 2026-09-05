#!/usr/bin/env python3
"""Full 2MB identity restore of the stock backup, verified sector-for-sector.

Uses boot_protection_bypass = False (identity/physical addressing) for both
write and verify, so the restored image matches the original working dump at
the physical level the chip actually boots and executes from. Skips sectors
that already match to save time.
"""
import sys
import time

import serial
from serial.tools import list_ports
from bk7231tools.serial import BK7231Serial

try:
    from bk7231tools.serial.base import EraseSize
except ImportError:
    from bk7231tools.serial.base.enums import EraseSize

MY_SERIAL = "5C94136993"
BRIDGE_EXIT = bytes([0xDE, 0xAD, 0xBE, 0xEF]) + b"BKJIG-EXIT42"
BACKUP = r"C:\Users\rose\bk7238-backups\t1-2s-nl_stock_full_2026-09-04.bin"


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


def main():
    img = open(BACKUP, "rb").read()
    print(f"backup: {len(img)} bytes")
    port = find_port()
    enter(port)
    bk = None
    try:
        bk = BK7231Serial(port, baudrate=115200, link_timeout=25.0, cmnd_timeout=3.0)
        bk.info = lambda *a: None
        bk.connect()
        bk.boot_protection_bypass = False
        bk.flash_unprotect()

        wrote = skipped = 0
        t0 = time.time()
        for off in range(0, len(img), 4096):
            want = img[off:off + 4096]
            cur = bk.flash_read_bytes(off, 4096, crc_check=False)
            if cur == want:
                skipped += 1
                continue
            bk.flash_erase_block(off, EraseSize.SECTOR_4K)
            bk.flash_write_4k(off, want, crc_check=True)
            wrote += 1
            if off % 0x20000 == 0:
                print(f"  0x{off:06X} ({off*100//len(img)}%) wrote={wrote} skip={skipped}", flush=True)
        print(f"done: wrote {wrote}, skipped {skipped} sectors in {time.time()-t0:.0f}s")

        # verify whole image
        bad = []
        for off in range(0, len(img), 4096):
            if bk.flash_read_bytes(off, 4096, crc_check=False) != img[off:off + 4096]:
                bad.append(off)
        print("VERIFY:", "OK, byte-exact" if not bad else f"{len(bad)} bad sectors, first 0x{bad[0]:X}")
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


if __name__ == "__main__":
    main()
