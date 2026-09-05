#!/usr/bin/env python3
"""Flash the OpenBeken full physical image (all_2M.1220.bin) at 0x0.

bk7231tools operates on RAW PHYSICAL flash (a full-chip dump is exactly the
flash size, CRC bytes included), so the app must be written in physical
(beken_onchip_crc) format. The packager's all_2M image is exactly that: the
matched v1.0.15 bootloader at phys 0x0 + the app at phys 0x11000. We write it
with boot_protection_bypass = False (identity/physical addressing) and verify
sector-by-sector, then reboot.

Usage: python tools/flash_obk_full.py [all_2M.1220.bin]
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
DEFAULT_IMG = r"C:\Users\rose\rc-switch-firmware\output\obk_rcswitch_full_2M.bin"


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
    img = open(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IMG, "rb").read()
    # pad up to a 4K boundary
    if len(img) % 4096:
        img += b"\xff" * (4096 - len(img) % 4096)
    print(f"image: {len(img)} bytes ({len(img)//4096} sectors) @ phys 0x0")
    port = find_port()
    enter(port)
    bk = None
    try:
        bk = BK7231Serial(port, baudrate=115200, link_timeout=25.0, cmnd_timeout=3.0)
        bk.info = lambda *a: None
        bk.connect()
        bk.boot_protection_bypass = False   # identity = physical addressing
        bk.flash_unprotect()

        wrote = skipped = 0
        t0 = time.time()
        for off in range(0, len(img), 4096):
            want = img[off:off + 4096]
            if bk.flash_read_bytes(off, 4096, crc_check=False) == want:
                skipped += 1
                continue
            bk.flash_erase_block(off, EraseSize.SECTOR_4K)
            bk.flash_write_4k(off, want, crc_check=False)
            wrote += 1
            if off % 0x20000 == 0:
                print(f"  0x{off:06X} ({off*100//len(img)}%) wrote={wrote} skip={skipped}", flush=True)
        print(f"done: wrote {wrote}, skipped {skipped} in {time.time()-t0:.0f}s")

        bad = [off for off in range(0, len(img), 4096)
               if bk.flash_read_bytes(off, 4096, crc_check=False) != img[off:off + 4096]]
        print("VERIFY:", "OK byte-exact" if not bad else f"{len(bad)} bad, first 0x{bad[0]:X}")
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
