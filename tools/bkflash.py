#!/usr/bin/env python3
"""Flash/dump the BK7238 through the BKJIG bridge.

Usage:
  python bkflash.py info
  python bkflash.py read <out.bin> [start_hex] [length_hex]   (default 0x0, 0x200000)
  python bkflash.py write <in.bin> [start_hex]

Flow: put jig into FLASHMODE (bridge + CEN pulses to catch BootROM),
then run bk7231tools over the transparent bridge at a fixed 115200,
then send the bridge-exit magic to restore command mode.
"""
import sys
import time

import serial
from serial.tools import list_ports
from bk7231tools.serial import BK7231Serial

MY_SERIAL = "5C94136993"   # Claude's ESP32-S3 only; 5C94136896 is off-limits
BRIDGE_EXIT = bytes([0xDE, 0xAD, 0xBE, 0xEF]) + b"BKJIG-EXIT42"


def find_port():
    for p in list_ports.comports():
        if p.serial_number and MY_SERIAL in p.serial_number:
            return p.device
    raise SystemExit(f"ERROR: jig board (USB serial {MY_SERIAL}) not found")


def enter_flashmode(port):
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
    if b"ENTERING BRIDGE" not in buf:
        raise SystemExit(f"ERROR: jig did not enter bridge mode (got: {buf!r})")
    print("jig: bridge mode entered")


def exit_bridge(port):
    s = serial.Serial(port, 115200, timeout=0.2)
    s.write(BRIDGE_EXIT)
    s.flush()
    time.sleep(0.5)
    s.close()
    print("jig: bridge exit magic sent (ESP restarting to command mode)")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    op = sys.argv[1].lower()
    port = find_port()
    enter_flashmode(port)

    bk = None
    try:
        bk = BK7231Serial(port, baudrate=115200, link_timeout=25.0, cmnd_timeout=3.0)
        bk.info = lambda *a: None  # silence per-4k-page spam; we print our own progress
        bk.connect()
        print(f"linked! chip info: {bk.read_chip_info()}")
        print(f"protocol: {bk.protocol_type.name if hasattr(bk, 'protocol_type') else '?'}")
        try:
            print(f"flash id: {bk.flash_read_id()}")
        except Exception as e:
            print(f"flash id unavailable: {e}")

        if op == "info":
            pass
        elif op == "read":
            out = sys.argv[2]
            start = int(sys.argv[3], 16) if len(sys.argv) > 3 else 0x0
            length = int(sys.argv[4], 16) if len(sys.argv) > 4 else 0x200000
            t0 = time.time()
            done = 0
            with open(out, "wb") as f:
                for chunk in bk.flash_read(start, length, crc_check=True):
                    f.write(chunk)
                    done += len(chunk)
                    if done % 0x10000 == 0:
                        pct = done / length * 100
                        rate = done / max(time.time() - t0, 0.001)
                        print(f"  read 0x{start+done:06X} ({pct:.0f}%, {rate/1024:.1f} KiB/s)", flush=True)
            print(f"saved {done} bytes to {out} in {time.time()-t0:.0f}s")
        elif op == "write":
            src = sys.argv[2]
            start = int(sys.argv[3], 16) if len(sys.argv) > 3 else 0x0
            data = open(src, "rb").read()
            bk.flash_unprotect()
            print(f"writing {len(data)} bytes at 0x{start:06X}")
            t0 = time.time()
            with open(src, "rb") as f:
                for addr in bk.program_flash(f, len(data), start, crc_check=True):
                    if addr % 0x10000 == 0:
                        print(f"  wrote 0x{addr:06X} ({(addr-start)/len(data)*100:.0f}%)", flush=True)
            print(f"write done in {time.time()-t0:.0f}s")
        elif op == "erase":
            # erase <start_hex> <length_hex> — 4K-aligned region to 0xFF
            start = int(sys.argv[2], 16)
            length = int(sys.argv[3], 16)
            if start & 0xFFF or length & 0xFFF:
                raise SystemExit("ABORT: erase region must be 4K-aligned")
            try:
                from bk7231tools.serial.base import EraseSize
            except ImportError:
                from bk7231tools.serial.base.enums import EraseSize
            bk.flash_unprotect()
            addr = start
            while addr < start + length:
                if addr % 0x10000 == 0 and start + length - addr >= 0x10000:
                    bk.flash_erase_block(addr, EraseSize.BLOCK_64K)
                    addr += 0x10000
                else:
                    bk.flash_erase_block(addr, EraseSize.SECTOR_4K)
                    addr += 0x1000
                if addr % 0x40000 == 0:
                    print(f"  erased to 0x{addr:06X}", flush=True)
            print(f"erased 0x{start:06X}..0x{start+length:06X}")
        elif op == "writetest":
            # Non-destructive write verification on a blank sector:
            # write pattern -> read back -> erase -> verify blank again.
            addr = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x180000
            before = bk.flash_read_bytes(addr, 4096)
            if before != b"\xff" * 4096:
                raise SystemExit(f"ABORT: sector 0x{addr:06X} is not blank, refusing to test there")
            pattern = bytes((i * 7 + (i >> 8)) & 0xFF for i in range(4096))
            bk.flash_unprotect()
            print("flash unprotected (SR protect bits cleared)")
            bk.flash_write_4k(addr, pattern, crc_check=True)
            readback = bk.flash_read_bytes(addr, 4096)
            print(f"write+readback at 0x{addr:06X}: {'MATCH' if readback == pattern else 'MISMATCH!'}")
            try:
                from bk7231tools.serial.base import EraseSize
            except ImportError:
                from bk7231tools.serial.base.enums import EraseSize
            bk.flash_erase_block(addr, EraseSize.SECTOR_4K)
            blank = bk.flash_read_bytes(addr, 4096)
            print(f"erase restore: {'BLANK OK' if blank == b'\xff'*4096 else 'NOT BLANK!'}")
        else:
            print(f"unknown op {op}")
    finally:
        if bk:
            try:
                bk.reboot_chip()  # leave the BK running its app, not the bootloader
            except Exception:
                pass
            try:
                bk.close()
            except Exception:
                pass
        exit_bridge(port)


if __name__ == "__main__":
    main()
