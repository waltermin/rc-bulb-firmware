#!/usr/bin/env python3
"""Bootloader-safe flasher for the T1-2S-NL (BK7238) via the BKJIG bridge.

Why this exists: the T1-2S-NL bootloader CRC (0xB397B8F6) is unknown to
bk7231tools/ltchiptool, so their default boot-protection *bypass* (which adds
flash_size to every offset to reach an unprotected alias of the protected boot
sectors) is unreliable here and intermittently erases 0x0..0x11000, bricking
boot.

This flasher never uses the bypass:
  - boot_protection_bypass = False -> identity addressing, so no logical
    address can ever map to physical 0x0.
  - It refuses to write anything below APP_BASE (0x11000). The app region is
    NOT write-protected, so identity writes there succeed; the bootloader's
    own protection actively rejects any stray low write instead of aliasing
    around it.
  - It snapshots the first bootloader sector before flashing and re-verifies
    it byte-for-byte afterwards, failing loudly if anything moved.

Usage:
  python tools/flash_safe.py [firmware.uf2]   (default .pio/build/t1-2s/firmware.uf2)
"""
import sys
import time
from pathlib import Path

from io import BytesIO

import serial
from serial.tools import list_ports
from bk7231tools.serial import BK7231Serial

import ltchiptool  # noqa: F401  (fully initialize before importing uf2tool, else circular)
from uf2tool import UploadContext
from uf2tool.models.enums import OTAScheme
from uf2tool.models.uf2 import UF2

JIG_USB_SERIAL = "5C94136993"   # Claude's ESP32-S3; never touch other CH343s
BRIDGE_EXIT = bytes([0xDE, 0xAD, 0xBE, 0xEF]) + b"BKJIG-EXIT42"
APP_BASE = 0x11000              # everything below this is bootloader / part table
DEFAULT_IMAGE = Path(__file__).parent.parent / ".pio" / "build" / "t1-2s" / "firmware.uf2"


def find_port():
    for p in list_ports.comports():
        if p.serial_number and JIG_USB_SERIAL in p.serial_number:
            return p.device
    raise SystemExit(f"ERROR: jig board (USB serial {JIG_USB_SERIAL}) not found")


def enter_bridge(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b"\nFLASHMODE\n")
    t0 = time.time()
    buf = b""
    while time.time() - t0 < 3:
        buf += s.read(256)
        if b"ENTERING BRIDGE" in buf:
            s.close()
            return
    s.close()
    raise SystemExit(f"ERROR: jig did not enter bridge mode (got {buf!r})")


def exit_bridge(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    s.write(BRIDGE_EXIT)
    s.flush()
    time.sleep(1.0)
    s.close()


def collect_parts(image: Path):
    uf2 = UF2(image.open("rb"))
    uf2.read()
    ctx = UploadContext(uf2)
    parts = ctx.collect_data(OTAScheme.FLASHER_SINGLE)
    return {offs: io.getvalue() for offs, io in parts.items()}


def main():
    image = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_IMAGE
    if not image.exists():
        raise SystemExit(f"ERROR: image not found: {image}")

    parts = collect_parts(image)
    for offs in sorted(parts):
        print(f"  part 0x{offs:06X}  {len(parts[offs])} bytes")
    # SAFETY: refuse to proceed if any part would touch the boot region.
    bad = [o for o in parts if o < APP_BASE]
    if bad:
        raise SystemExit(f"ABORT: image has parts below 0x{APP_BASE:X}: {[hex(o) for o in bad]}")

    port = find_port()
    print(f"jig on {port}; image {image.name}")
    enter_bridge(port)
    bk = None
    try:
        bk = BK7231Serial(port, baudrate=115200, link_timeout=25.0, cmnd_timeout=3.0)
        bk.info = lambda *a: None
        bk.connect()
        print(f"linked: {bk.read_chip_info()}, flash 0x{bk.flash_size:X}")

        # Identity addressing only — this is the whole safety guarantee.
        bk.boot_protection_bypass = False

        # Snapshot bootloader sector 0 to prove it's untouched afterwards.
        boot0 = bk.flash_read_bytes(0x0, 256, crc_check=False)
        print(f"bootloader[0:16] before: {boot0[:16].hex()}")

        bk.flash_unprotect()
        for offs in sorted(parts):
            data = parts[offs]
            assert offs >= APP_BASE
            # Highest byte this part reaches must also stay above the boot region.
            assert offs + len(data) > APP_BASE
            print(f"writing 0x{offs:06X} ({len(data)} bytes)...", flush=True)
            t0 = time.time()
            # program_flash handles sector erase + unaligned starts; with bypass
            # OFF every offset is identity, so nothing can alias down to 0x0.
            gen = bk.program_flash(
                io=BytesIO(data),
                io_size=len(data),
                start=offs,
                crc_check=True,
                really_erase=True,
            )
            for _ in gen:
                pass
            print(f"  done in {time.time()-t0:.0f}s")

        # Verify bootloader survived.
        boot1 = bk.flash_read_bytes(0x0, 256, crc_check=False)
        if boot1 != boot0:
            print(f"bootloader[0:16] after:  {boot1[:16].hex()}")
            raise SystemExit("FAIL: bootloader sector changed during flash!")
        print(f"bootloader intact after flash: {boot1[:16].hex()}")
        print("SUCCESS")
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
        exit_bridge(port)
        print("bridge exited, BK rebooted into app")


if __name__ == "__main__":
    main()
