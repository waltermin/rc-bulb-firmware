#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial", "bk7231tools==2.1.2"]
# ///
"""Drive the BKJIG2 harness (jig2/, ESP32-S3 on native USB) to read/write the
T1-2S-NL's BK7238 flash.

Usage (uv fetches the deps on first run):
  uv run tools/bkjig2.py ping
  uv run tools/bkjig2.py power on|off|cycle
  uv run tools/bkjig2.py detect                 # find which of GPIO5/6 is TX vs RX
  uv run tools/bkjig2.py catchmode cen|power    # reset via CEN (GPIO7) or relay power cycle
  uv run tools/bkjig2.py swap                   # flip the saved TX/RX order by hand
  uv run tools/bkjig2.py console [secs]         # watch the target's UART output
  uv run tools/bkjig2.py info                   # link to the BootROM, print chip/flash
  uv run tools/bkjig2.py read <out.bin> [start_hex] [length_hex]   (default 0x0 0x200000)
  uv run tools/bkjig2.py write <in.bin> [start_hex] --yes
  uv run tools/bkjig2.py flash-full <all_2M.bin> --yes  # whole physical image at 0x0,
                                                        # bootloader included
  uv run tools/bkjig2.py factory --id 65 --channel 4 [--image output/rcsw_bare_full_2M.bin] [--backup]
      # production: full flash (verified) -> settings sector -> boot check, timed
      # (--backup first saves the stock flash to output/factory/, ~3 min more)
  uv run tools/bkjig2.py test --id 64 [--channel 4] [--wait]
      # over-the-air relay/LED test pattern for a finished device; runs on the
      # jig in the background, so the next unit can be flashed meanwhile
  uv run tools/bkjig2.py bench-test --id 73 [--channel 4]
      # the same over-the-air test on the unit ON the jig, checked automatically
      # against its own serial log (PASS/FAIL)
  uv run tools/bkjig2.py hold --relay 64-73 [--led 64,66] [--channel 4] | hold --off
  uv run tools/bkjig2.py hold --bulbs 0-15 --color 0,0,0,0,0    # bulbs: LightUpdateV2 r,g,b,ww,cw
      # keep relays/LEDs ON over the air (devices fall back to off after ~15 s of
      # silence, so the jig re-broadcasts until `hold --off`)
  uv run tools/bkjig2.py probe --ids 64-115 [--channel 4] [--timeout 35]
      # DFU "ping": put each unit into DFU2 mode one at a time, log its hello and
      # ICMP-ping it, then release it WITHOUT an image (its flash is never touched)
  uv run tools/bkjig2.py ota <image.rbl> --version rc_marker [--id 5] [--watch 240]
      # Wi-Fi only: the jig serves the .rbl from a soft AP and broadcasts a 0x05
      # Dfu2Request; serial is used only to WATCH the target's log for ver=<version>

Flash flow: FLASHMODE makes the jig reset the target (CEN pulse, or a relay
power cycle after `CATCHMODE POWER`) and catch its BootROM itself, then bridge
USB <-> target UART; bk7231tools runs over that bridge at a fixed 115200;
finally the exit magic returns the jig to command mode.

Bootloader safety (see tools/flash_safe.py for the history): this T1-2S-NL's
bootloader (CRC 0xB397B8F6) is unknown to bk7231tools, and its default
boot-protection *bypass* (offset + flash_size aliasing) hits the wrong
physical sectors on this chip and has bricked boot before. So every operation
here uses identity addressing (boot_protection_bypass = False), writes below
APP_BASE (0x11000) are refused, and the first boot sector is snapshotted and
re-verified around every write.
"""
import argparse
import hashlib
import os
import re
import struct
import zlib
import subprocess
import sys
import time

import serial
from serial.tools import list_ports

JIG_VID = 0x303A                     # Espressif native USB (USB-Serial-JTAG)
JIG_SERIAL = "14:C1:9F:42:8A:54"     # the S3's MAC, reported as its USB serial
BRIDGE_EXIT = bytes([0xDE, 0xAD, 0xBE, 0xEF]) + b"BKJIG-EXIT42"
APP_BASE = 0x11000                   # below this: bootloader + partition table
SETTINGS_ADDR = 0x1FF000             # fw/main/settings.c RCSW_SETTINGS_ADDR


def find_port():
    for p in list_ports.comports():
        if p.vid == JIG_VID and p.serial_number == JIG_SERIAL:
            return p.device
    sys.exit(f"ERROR: jig (USB serial {JIG_SERIAL}) not found; pass --port")


def open_port(port):
    # The native USB port doesn't reset the S3 on open, so no settle delay.
    s = serial.Serial(port, 115200, timeout=0.1)
    s.reset_input_buffer()
    return s


def command(port, line, until, timeout):
    """Send one command line, echo the jig's replies, return when a reply line
    contains any string in `until` (or raise on timeout)."""
    s = open_port(port)
    try:
        s.write(line.encode() + b"\n")
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            buf += s.read(256)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").strip()
                if not text:
                    continue
                print(f"  jig: {text}")
                if any(u in text for u in until):
                    return text
        raise TimeoutError(f"no reply to {line!r} within {timeout}s")
    finally:
        s.close()


def exit_bridge(port):
    s = open_port(port)
    try:
        s.write(BRIDGE_EXIT)
        s.flush()
        deadline = time.time() + 2
        buf = b""
        while time.time() < deadline and b"BRIDGE EXIT" not in buf:
            buf += s.read(256)
    finally:
        s.close()
    print("jig: back in command mode")


def console(port, secs):
    command(port, "BRIDGE", ["BRIDGE OK"], 3)
    s = open_port(port)
    try:
        end = time.time() + secs
        while time.time() < end:
            data = s.read(512)
            if data:
                sys.stdout.write(data.decode(errors="replace"))
                sys.stdout.flush()
    finally:
        s.close()
        print()
        exit_bridge(port)


BAUD = 115200  # flash-link rate after linking; set by --baud


def with_bootrom(port, fn):
    reply = command(port, "FLASHMODE", ["ENTERING BRIDGE", "FLASHMODE FAIL"], 20)
    if "FAIL" in reply:
        sys.exit("ERROR: BootROM did not answer (run `detect` first, check wiring)")

    from bk7231tools.serial import BK7231Serial

    bk = None
    try:
        # The jig already holds the chip in download mode, so linking is quick.
        # Links at 115200, then (if BAUD differs) sends SetBaudRate; the jig's
        # bridge follows the switch on its side of the UART.
        bk = BK7231Serial(port, baudrate=BAUD, link_timeout=5.0, cmnd_timeout=3.0,
                          link_baudrate=115200)
        bk.info = lambda *a: None  # silence per-page spam; we print our own progress
        bk.connect()
        # connect() runs flash_detect_size(), which re-enables the bypass on exit.
        bk.boot_protection_bypass = False
        print(f"linked: chip {bk.chip_type.name if bk.chip_type else '?'}, "
              f"bootloader {bk.bootloader_type.name if bk.bootloader_type else 'unknown'}, "
              f"flash {bk.flash_size // 1024 if bk.flash_size else '?'} KiB, {BAUD} baud")
        fn(bk)
    finally:
        if bk:
            try:
                bk.reboot_chip()  # leave the target running its app
            except Exception:
                pass
            bk.close()
        exit_bridge(port)


def do_read(bk, out, start, length):
    t0 = time.time()
    done = 0
    with open(out, "wb") as f:
        for chunk in bk.flash_read(start, length, crc_check=True):
            f.write(chunk)
            done += len(chunk)
            if done % 0x10000 == 0:
                rate = done / max(time.time() - t0, 0.001)
                print(f"  read 0x{start + done:06X} ({done / length * 100:.0f}%, "
                      f"{rate / 1024:.1f} KiB/s)", flush=True)
    print(f"saved {done} bytes to {out} in {time.time() - t0:.0f}s")


def do_write(bk, src, start):
    data = open(src, "rb").read()
    boot_before = bk.flash_read_bytes(0, 0x1000)
    bk.flash_unprotect()
    print(f"writing {len(data)} bytes at 0x{start:06X}")
    t0 = time.time()
    with open(src, "rb") as f:
        for addr in bk.program_flash(f, len(data), start, crc_check=True):
            if addr % 0x10000 == 0:
                print(f"  wrote 0x{addr:06X} ({(addr - start) / len(data) * 100:.0f}%)", flush=True)
    print(f"write done in {time.time() - t0:.0f}s (CRC-checked per block)")
    if bk.flash_read_bytes(0, 0x1000) != boot_before:
        sys.exit("ERROR: boot sector 0x0 CHANGED during the write -- do not power-cycle; "
                 "restore it with tools/recover_boot.py from a full backup")
    print("boot sector 0x0 unchanged")


def do_flash_full(bk, src):
    """Write a full physical image (bootloader at 0x0 + app): erase + write every
    4K sector, then a byte-exact read-back of the whole image. (No per-sector
    pre-read: on factory units every sector differs, and it cost ~1/3 of the time.)"""
    try:
        from bk7231tools.serial.base import EraseSize
    except ImportError:
        from bk7231tools.serial.base.enums import EraseSize

    img = open(src, "rb").read()
    if len(img) % 4096:
        img += b"\xff" * (4096 - len(img) % 4096)
    print(f"image: {len(img)} bytes ({len(img) // 4096} sectors) @ phys 0x0")
    bk.flash_unprotect()

    t0 = time.time()
    for off in range(0, len(img), 4096):
        bk.flash_erase_block(off, EraseSize.SECTOR_4K)
        bk.flash_write_4k(off, img[off:off + 4096], crc_check=False)
        if off % 0x20000 == 0:
            print(f"  0x{off:06X} ({off * 100 // len(img)}%)", flush=True)
    print(f"done: wrote {len(img) // 4096} sectors in {time.time() - t0:.0f}s")

    bad = [off for off in range(0, len(img), 4096)
           if bk.flash_read_bytes(off, 4096, crc_check=False) != img[off:off + 4096]]
    if bad:
        sys.exit(f"VERIFY FAILED: {len(bad)} sectors differ, first 0x{bad[0]:X} -- "
                 "do not power-cycle; re-run flash-full")
    print("VERIFY: OK, byte-exact")


def ota(port, rbl, version, target_id, watch):
    data = open(rbl, "rb").read()
    if data[:4] != b"RBL\0":
        sys.exit(f"{rbl} is not an rt_ota .rbl image")
    build_id = hashlib.sha256(version.encode()).digest()[:8].hex()
    print(f"image {rbl}: {len(data)} bytes, version {version!r}, build id {build_id}")

    # Stage the image in the S3's spare flash (outside its partition table).
    subprocess.run(["esptool", "--port", port, "--after", "hard-reset", "write-flash",
                    "0x800000", rbl], check=True, stdout=subprocess.DEVNULL)
    deadline = time.time() + 15
    while time.time() < deadline:   # the S3 re-enumerates after the reset
        time.sleep(0.5)
        try:
            command(port, "PING", ["BKJIG2"], 2)
            break
        except Exception:
            continue

    command(port, f"DFU2 START {len(data)} {build_id} 0x02 {target_id} 0",
            ["DFU2 serving", "DFU2 ERR", "ERR"], 30)

    # Watch the target's UART (and the jig's DFU2 events) until it reboots into
    # the new build. Serial is read-only here; the update goes over Wi-Fi.
    command(port, "BRIDGE", ["BRIDGE OK"], 3)
    s = open_port(port)
    marker = f"ver={version}".encode()
    seen = b""
    ok = False
    try:
        end = time.time() + watch
        while time.time() < end:
            chunk = s.read(4096)
            if chunk:
                sys.stdout.write(chunk.decode(errors="replace"))
                sys.stdout.flush()
                seen = (seen + chunk)[-4096:]
                if marker in seen:
                    ok = True
                    time.sleep(2)   # let the banner finish
                    sys.stdout.write(s.read(4096).decode(errors="replace"))
                    break
    finally:
        s.close()
        print()
        exit_bridge(port)
        command(port, "DFU2 STOP", ["DFU2 STOP OK"], 5)
    print(f"\nRESULT: {'UPDATED -- target booted ' + version if ok else 'no ' + marker.decode() + ' seen within ' + str(watch) + 's'}")


def settings_sector(dev_id, channel, default_relay=0, default_led=0, fallback_ms=15000):
    """The 4 KB settings sector exactly as fw/main/settings.c reads it: magic "RCS1",
    version 1, {id, channel, default_relay, default_led, fallback_ms}, CRC32."""
    body = struct.pack("<II BBBB I", 0x31534352, 1, dev_id, channel,
                       default_relay, default_led, fallback_ms)
    rec = body + struct.pack("<I", zlib.crc32(body))
    return rec + b"\xff" * (4096 - len(rec))


def boot_check(port, secs=10):
    """Power-cycle the target and return its boot log (serial is read-only here)."""
    command(port, "POWER CYCLE 1000", ["POWER on OK"], 5)
    command(port, "BRIDGE", ["BRIDGE OK"], 3)
    s = open_port(port)
    log = b""
    try:
        end = time.time() + secs
        while time.time() < end:
            log += s.read(4096)
            if b"monitor started" in log:
                time.sleep(0.5)
                log += s.read(4096)
                break
    finally:
        s.close()
        exit_bridge(port)
    return log.decode(errors="replace")


def factory(port, image, dev_id, channel, backup):
    img_size = os.path.getsize(image)
    print(f"factory: image {image} ({img_size} bytes) -> id {dev_id}, channel {channel}")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    t0 = time.time()
    marks = []

    def mark(step):
        marks.append((step, time.time()))

    def run(bk):
        mark("link")
        if bk.chip_type is None or bk.chip_type.name != "BK7238":
            sys.exit(f"ABORT: expected a BK7238, linked {bk.chip_type}")
        if backup:
            os.makedirs("output/factory", exist_ok=True)
            out = f"output/factory/{stamp}-id{dev_id}-stock.bin"
            print(f"[1/3] backing up stock flash -> {out}")
            do_read(bk, out, 0, bk.flash_size)
            mark("backup")
        else:
            print("[1/3] backup skipped")
        print("[2/3] flashing production image")
        do_flash_full(bk, image)
        mark("flash+verify")
        print(f"[3/3] writing settings sector @0x{SETTINGS_ADDR:X} (id {dev_id}, channel {channel})")
        sector = settings_sector(dev_id, channel)
        bk.flash_erase_block(SETTINGS_ADDR, EraseSizeOf().SECTOR_4K)
        bk.flash_write_4k(SETTINGS_ADDR, sector, crc_check=False)
        if bk.flash_read_bytes(SETTINGS_ADDR, 4096, crc_check=False) != sector:
            sys.exit("ABORT: settings sector read-back mismatch")
        print("settings sector verified")
        mark("settings")

    with_bootrom(port, run)

    print("boot check: power-cycling and reading the boot log")
    log = boot_check(port)
    mark("boot check")
    for line in log.splitlines():
        if any(k in line for k in ("BK7238_", "settings:", "rc-switch (bare", "monitor started")):
            print("  " + line.strip())
    banner = re.search(r"rc-switch \(bare SDK\) ver=(\S+) id=(\d+) chan=(\d+)", log)
    if not banner or int(banner.group(2)) != dev_id or int(banner.group(3)) != channel:
        sys.exit(f"\nFACTORY FAIL: boot banner did not report id={dev_id} chan={channel}")
    prev = t0
    for step, t in marks:
        print(f"  {step:<13} {t - prev:6.1f}s")
        prev = t
    print(f"\nFACTORY OK: {banner.group(1)} running as id {dev_id} on channel {channel} "
          f"in {time.time() - t0:.0f}s total")


def EraseSizeOf():
    try:
        from bk7231tools.serial.base import EraseSize
    except ImportError:
        from bk7231tools.serial.base.enums import EraseSize
    return EraseSize


# Mirrors STEPS in jig2/main/airtest.c: (seconds, what the device should do).
AIRTEST_TIMELINE = [
    (8, "relay clicks ON / OFF twice (2 s each), LED off"),
    (3, "relay ON + LED ON"),
    (2, "both OFF -- test over"),
]


def airtest(port, dev_id, channel, wait):
    reply = command(port, f"AIRTEST {dev_id} {channel}", ["AIRTEST started", "ERR"], 5)
    if "ERR" in reply:
        sys.exit(f"test not started: {reply}")
    print(f"testing id {dev_id} on channel {channel} -- watch the device:")
    t = 0
    for secs, what in AIRTEST_TIMELINE:
        print(f"  {t:>2}-{t + secs:<2}s  {what}")
        t += secs
    if not wait:
        print("(running on the jig in the background; flashing can continue)")
        return
    t0 = time.time()
    for secs, what in AIRTEST_TIMELINE:
        print(f"  now: {what}", flush=True)
        time.sleep(secs)
    print(f"done in {time.time() - t0:.0f}s")


def bench_test(port, dev_id, channel):
    """Run the air test on the unit that is on the jig and judge it from the
    unit's own status lines (~1/s): packet counters and relay/LED outputs."""
    # The jig buffers the unit's UART output while not bridging; drain that
    # backlog first so old status lines can't count toward this test.
    command(port, "BRIDGE", ["BRIDGE OK"], 3)
    try:
        s = open_port(port)
        quiet = time.time()
        while time.time() - quiet < 0.3:
            if s.read(4096):
                quiet = time.time()
        s.close()
    finally:
        exit_bridge(port)
    reply = command(port, f"AIRTEST {dev_id} {channel}", ["AIRTEST started", "ERR"], 5)
    if "ERR" in reply:
        sys.exit(f"test not started: {reply}")
    command(port, "BRIDGE", ["BRIDGE OK"], 3)
    raw = b""
    try:
        s = open_port(port)
        end = time.time() + sum(secs for secs, _ in AIRTEST_TIMELINE) + 3
        while time.time() < end:
            raw += s.read(4096)
        s.close()
    finally:
        exit_bridge(port)

    log = raw.decode(errors="replace")
    ids = re.findall(r"rc-switch \(bare SDK\) ver=\S+ id=(\d+)", log)
    samples = [(int(m[0]), int(m[1]), int(m[2]), int(m[3]))
               for m in re.findall(r"relay=(\d+) led=(\d+) dfu=\d+ out\(r=(\d),l=(\d)\)", log)]
    if not samples:
        sys.exit("BENCH TEST FAIL: no status lines from the unit (is it on the jig and running rc-switch?)")
    states = [(r, l) for _, _, r, l in samples]
    collapsed = [st for i, st in enumerate(states) if i == 0 or st != states[i - 1]]
    relay_pkts = samples[-1][0] - samples[0][0]
    led_pkts = samples[-1][1] - samples[0][1]
    print("relay/LED as the unit reported them (1/s): " + " ".join(f"{r}{l}" for r, l in states))

    checks = {
        "received relay packets (>=100)": relay_pkts >= 100,
        "received LED packets (>=100)": led_pkts >= 100,
        "relay clicked on twice (LED off)": sum(1 for st in collapsed if st == (1, 0)) >= 2,
        "relay + LED both on": (1, 1) in states,
        "ended with both off": states[-1] == (0, 0),
    }
    for name, ok in checks.items():
        print(f"  {'ok  ' if ok else 'FAIL'} {name}")
    print(f"  packets: {relay_pkts} relay, {led_pkts} LED")
    if ids and int(ids[-1]) != dev_id:
        print(f"  note: the unit on the jig reports id {ids[-1]}, not {dev_id}")
    if not all(checks.values()):
        sys.exit(f"\nBENCH TEST FAIL: id {dev_id}")
    print(f"\nBENCH TEST PASS: id {dev_id}")


def parse_ids(spec):
    """'64-73,80' -> {64, ..., 73, 80}; 'none' -> {} (hold everything OFF)"""
    ids = set()
    if spec.strip().lower() == "none":
        return ids
    for part in spec.split(","):
        lo, _, hi = part.strip().partition("-")
        ids.update(range(int(lo), int(hi or lo) + 1))
    if any(not 0 <= i <= 255 for i in ids):
        sys.exit("ids must be 0..255")
    return ids


def bitfield_hex(ids):
    bits = bytearray(32)
    for i in ids:
        bits[i // 8] |= 1 << (i % 8)
    return bits.hex()


BULB_ENTRIES_PER_PACKET = 11   # LightUpdateV2 limit (ESP8266 bulbs drop larger frames)
RAW_SLOTS = 8


def hold(port, relay, led, bulbs, color, channel, off):
    if off:
        command(port, "HOLD OFF", ["HOLD OFF OK"], 3)
        print("stopped; devices fall back to their defaults within ~15 s")
        return
    if relay is None and led is None and bulbs is None:
        sys.exit("give --relay, --led and/or --bulbs ids, or --off")
    if bulbs is not None:
        ids = sorted(parse_ids(bulbs))
        rgbcw = [int(x) for x in color.split(",")]
        if len(rgbcw) != 5 or any(not 0 <= v <= 255 for v in rgbcw):
            sys.exit("--color needs r,g,b,ww,cw each 0..255")
        chunks = [ids[i:i + BULB_ENTRIES_PER_PACKET] for i in range(0, len(ids), BULB_ENTRIES_PER_PACKET)]
        if len(chunks) > RAW_SLOTS:
            sys.exit(f"at most {RAW_SLOTS * BULB_ENTRIES_PER_PACKET} bulbs per hold")
        command(port, "HOLD RAW OFF", ["HOLD RAW OFF OK"], 3)
        for slot, chunk in enumerate(chunks):
            body = bytes([0x03, len(chunk)]) + b"".join(bytes([i] + rgbcw) for i in chunk)
            command(port, f"HOLD RAW {slot} {body.hex()} {channel}", ["HOLD RAW", "ERR"], 3)
        print(f"holding bulbs {bulbs} at r,g,b,ww,cw={color} on channel {channel} "
              f"({len(chunks)} LightUpdateV2 packets)")
    for kind, spec in (("RELAY", relay), ("LED", led)):
        if spec is not None:
            ids = parse_ids(spec)
            command(port, f"HOLD {kind} {bitfield_hex(ids)} {channel}", ["HOLD", "ERR"], 3)
            print(f"holding {kind.lower()} ON for {len(ids)} ids ({spec}) on channel {channel}")


def probe(port, ids_spec, channel, timeout):
    import serial as pyserial
    ids = sorted(parse_ids(ids_spec))
    start_cmd = "DFU2 START 0 0000000000000000 0x02 {} 0 RC-Update rcupdate1 " + str(channel)
    conn = {"s": None, "buf": b""}

    def connect(first_id):
        """(Re)open the jig and (re)start the probe server; survives jig resets."""
        deadline = time.time() + 60
        while True:
            try:
                p = find_port_quiet()
                if p:
                    conn["s"] = open_port(p)
                    conn["s"].timeout = 0.1
                    conn["buf"] = b""
                    reply = send(start_cmd.format(first_id), ["DFU2 serving", "ERR"], 15)
                    if "already running" in reply:
                        send("DFU2 STOP", ["DFU2 STOP OK"])
                        reply = send(start_cmd.format(first_id), ["DFU2 serving", "ERR"], 15)
                    if "ERR" in reply:
                        sys.exit(f"probe not started: {reply}")
                    return
            except (OSError, pyserial.SerialException, TimeoutError):
                pass
            if time.time() > deadline:
                sys.exit("jig did not come back within 60 s")
            time.sleep(1)

    def lines(until, secs):
        end = time.time() + secs
        while time.time() < end:
            conn["buf"] += conn["s"].read(4096)
            while b"\n" in conn["buf"]:
                raw, conn["buf"] = conn["buf"].split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    yield line
                    if until(line):
                        return

    def send(cmd, expect, secs=10):
        conn["s"].write(cmd.encode() + b"\n")
        for line in lines(lambda l: any(e in l for e in expect), secs):
            if any(e in line for e in expect):
                return line
        raise TimeoutError(f"no reply to {cmd!r}")

    def probe_one(dev_id):
        send(f"DFU2 TARGET {dev_id} 0", ["DFU2 TARGET OK"])
        t0 = time.time()
        r = {"hello": None, "ping": None, "ip": None}
        for line in lines(lambda l: f"DFU2 PING id={dev_id}:" in l, timeout):
            if line.startswith("DFU2 client connected from"):
                r["ip"] = line.rsplit(" ", 1)[-1]
            elif line.startswith("DFU2 hello:") and f" id={dev_id} " in line + " ":
                r["hello"] = time.time() - t0
                r["build"] = line.rsplit("build=", 1)[-1]
            elif line.startswith(f"DFU2 PING id={dev_id}:"):
                r["ping"] = line.split(": ", 1)[1]
        return r

    results = {}
    connect(ids[0])
    print(f"probing {len(ids)} ids on channel {channel} (fake build id, no image served)", flush=True)
    try:
        for dev_id in ids:
            for attempt in (1, 2):
                try:
                    r = probe_one(dev_id)
                    break
                except (OSError, pyserial.SerialException, TimeoutError) as e:
                    print(f"  (jig connection lost at id {dev_id}: {type(e).__name__}; "
                          f"reconnecting, attempt {attempt})", flush=True)
                    try:
                        conn["s"].close()
                    except Exception:
                        pass
                    time.sleep(2)
                    connect(dev_id)
            else:
                r = {"hello": None}
            results[dev_id] = r
            if r["hello"] is None:
                print(f"  id {dev_id:>3}: NO RESPONSE in {timeout:.0f}s", flush=True)
            else:
                print(f"  id {dev_id:>3}: DFU hello after {r['hello']:4.1f}s from {r['ip']}, "
                      f"build {r.get('build', '?')}, ping {r['ping'] or 'none'}", flush=True)
    finally:
        try:
            send("DFU2 STOP", ["DFU2 STOP OK"])
        except Exception:
            pass
        try:
            conn["s"].close()
        except Exception:
            pass
    ok = [i for i, r in results.items() if r["hello"] is not None]
    bad = [i for i, r in results.items() if r["hello"] is None]
    print(f"\nresponded: {len(ok)}/{len(results)}", flush=True)
    if bad:
        print("no response: " + ", ".join(map(str, bad)), flush=True)


def find_port_quiet():
    for p in list_ports.comports():
        if p.vid == JIG_VID and p.serial_number == JIG_SERIAL:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200,
                    help="flash-link baud after linking (the jig follows the switch)")
    sub = ap.add_subparsers(dest="op", required=True)
    sub.add_parser("ping")
    p = sub.add_parser("power"); p.add_argument("state", choices=["on", "off", "cycle"])
    sub.add_parser("detect")
    sub.add_parser("swap")
    p = sub.add_parser("catchmode"); p.add_argument("mode", choices=["cen", "power"])
    p = sub.add_parser("console"); p.add_argument("secs", type=float, nargs="?", default=10)
    sub.add_parser("info")
    p = sub.add_parser("read")
    p.add_argument("out")
    p.add_argument("start", nargs="?", default="0x0")
    p.add_argument("length", nargs="?", default="0x200000")
    p = sub.add_parser("write")
    p.add_argument("src")
    p.add_argument("start", nargs="?", default="0x0")
    p.add_argument("--yes", action="store_true", help="required: confirms overwriting target flash")
    p = sub.add_parser("factory")
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--channel", type=int, required=True)
    p.add_argument("--image", default="output/rcsw_bare_full_2M.bin")
    p.add_argument("--backup", action="store_true", help="save the stock flash first (~3 min)")
    p = sub.add_parser("test")
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--channel", type=int, default=4)
    p.add_argument("--wait", action="store_true", help="stay and call out each phase")
    p = sub.add_parser("bench-test")
    p.add_argument("--id", type=int, required=True)
    p.add_argument("--channel", type=int, default=4)
    p = sub.add_parser("hold")
    p.add_argument("--relay", help="ids to hold relay ON, e.g. 64-73,80")
    p.add_argument("--led", help="ids to hold LED ON ('none' = hold all LEDs OFF)")
    p.add_argument("--bulbs", help="bulb ids to hold at --color (LightUpdateV2)")
    p.add_argument("--color", default="0,0,0,0,0", help="r,g,b,ww,cw 0..255 (default all 0)")
    p.add_argument("--channel", type=int, default=4)
    p.add_argument("--off", action="store_true", help="stop holding")
    p = sub.add_parser("probe")
    p.add_argument("--ids", required=True, help="e.g. 64-115")
    p.add_argument("--channel", type=int, default=4)
    p.add_argument("--timeout", type=float, default=35, help="seconds to wait per id")
    p = sub.add_parser("ota")
    p.add_argument("rbl")
    p.add_argument("--version", required=True, help="USER_SW_VER the .rbl was built with")
    p.add_argument("--id", type=int, default=5, help="target device id")
    p.add_argument("--watch", type=float, default=240, help="seconds to watch for the new banner")
    p = sub.add_parser("flash-full")
    p.add_argument("src")
    p.add_argument("--yes", action="store_true", help="required: overwrites the bootloader too")
    a = ap.parse_args()

    global BAUD
    BAUD = a.baud
    port = a.port or find_port()
    if a.op == "ping":
        command(port, "PING", ["BKJIG2"], 3)
    elif a.op == "power":
        command(port, f"POWER {a.state.upper()}", ["POWER", "ERR"], 15)
    elif a.op == "detect":
        command(port, "DETECT", ["DETECT OK", "DETECT FAIL"], 30)
    elif a.op == "swap":
        command(port, "SWAP", ["SWAP OK"], 3)
    elif a.op == "catchmode":
        command(port, f"CATCHMODE {a.mode.upper()}", ["CATCHMODE", "ERR"], 3)
    elif a.op == "console":
        console(port, a.secs)
    elif a.op == "info":
        with_bootrom(port, lambda bk: None)
    elif a.op == "read":
        with_bootrom(port, lambda bk: do_read(bk, a.out, int(a.start, 16), int(a.length, 16)))
    elif a.op == "write":
        if not a.yes:
            sys.exit("refusing to write target flash without --yes")
        if int(a.start, 16) < APP_BASE:
            sys.exit(f"refusing to write below 0x{APP_BASE:X} (bootloader region)")
        with_bootrom(port, lambda bk: do_write(bk, a.src, int(a.start, 16)))
    elif a.op == "factory":
        if not (0 <= a.id <= 255 and 1 <= a.channel <= 14):
            sys.exit("id must be 0..255 and channel 1..14")
        factory(port, a.image, a.id, a.channel, a.backup)
    elif a.op == "test":
        airtest(port, a.id, a.channel, a.wait)
    elif a.op == "bench-test":
        bench_test(port, a.id, a.channel)
    elif a.op == "hold":
        hold(port, a.relay, a.led, a.bulbs, a.color, a.channel, a.off)
    elif a.op == "probe":
        probe(port, a.ids, a.channel, a.timeout)
    elif a.op == "ota":
        ota(port, a.rbl, a.version, a.id, a.watch)
    elif a.op == "flash-full":
        if not a.yes:
            sys.exit("refusing to overwrite the bootloader without --yes")
        with_bootrom(port, lambda bk: do_flash_full(bk, a.src))


if __name__ == "__main__":
    main()
