#!/usr/bin/env python3
"""Recover the BK7238 bootloader region (0x0..0x11000) using IDENTITY addressing.

The T1-2S-NL bootloader CRC is unknown to bk7231tools, so its automatic
boot-protection *bypass* (which writes to an aliased address = offset +
flash_size) is unreliable here: writes/verifies through the alias can succeed
while the chip still boots from an erased physical 0x0.

This writes and verifies the boot region with boot_protection_bypass = FALSE
(identity addressing = real physical sectors), after clearing the flash status-
register protection so the low sectors are directly writable. Verify reads use
the same identity path, so a pass means the bytes the chip actually boots from
are correct.

Usage: python tools/recover_boot.py [stock_boot.bin]
  stock_boot.bin = first 0x11000 bytes of the stock full-flash backup.
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
DEFAULT_BOOT = r"C:\Users\rose\AppData\Local\Temp\stock_boot.bin"


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
    boot = open(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BOOT, "rb").read()
    assert len(boot) % 4096 == 0, "boot image must be a whole number of 4K sectors"
    port = find_port()
    print(f"boot image: {len(boot)} bytes ({len(boot)//4096} sectors)")
    enter(port)
    bk = None
    try:
        bk = BK7231Serial(port, baudrate=115200, link_timeout=25.0, cmnd_timeout=3.0)
        bk.info = lambda *a: None
        bk.connect()
        bk.boot_protection_bypass = False   # identity addressing = physical sectors
        bk.flash_unprotect()                # clear SR BP bits so low sectors write

        for off in range(0, len(boot), 4096):
            bk.flash_erase_block(off, EraseSize.SECTOR_4K)
            bk.flash_write_4k(off, boot[off:off + 4096], crc_check=True)

        # Verify EVERY sector via the identity path (what the chip boots from).
        bad = []
        for off in range(0, len(boot), 4096):
            rb = bk.flash_read_bytes(off, 4096, crc_check=False)
            if rb != boot[off:off + 4096]:
                bad.append(off)
        if bad:
            print(f"FAIL: {len(bad)} sector(s) mismatch, first at 0x{bad[0]:X}")
            print(f"  physical 0x0 head: {bk.flash_read_bytes(0,16,crc_check=False).hex()}")
            sys.exit(1)
        print(f"SUCCESS: boot region verified byte-exact (identity), 0x0 head "
              f"{bk.flash_read_bytes(0,16,crc_check=False).hex()}")
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
