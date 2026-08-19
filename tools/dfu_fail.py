#!/usr/bin/env python3
"""DFU failure-injection tester.

Triggers DFU on bulb id 1 via the base station (COM9), reads the bulb's IP from
the running tail logfile, then pushes a firmware image in a deliberately broken
way to exercise the bulb's abort/rollback paths.

    python3 tools/dfu_fail.py <half|corrupt|slow> [image.bin]

Requires the no-reset tail (tools/tail.py) to be running and writing dfu-test.log
so we can observe the bulb's DHCP IP without opening its serial port.
"""
import hashlib
import os
import re
import socket
import struct
import sys
import time

import serial

MODE = sys.argv[1] if len(sys.argv) > 1 else "half"
IMG = sys.argv[2] if len(sys.argv) > 2 else "build/bulb-firmware.bin"
LOG = "dfu-test.log"
MAGIC = 0x686C4352
PORT = 3333


def trigger_dfu_and_get_ip():
    start = os.path.getsize(LOG) if os.path.exists(LOG) else 0
    base = serial.Serial("COM9", 115200, timeout=0.1)
    base.setDTR(False); base.setRTS(True); time.sleep(0.1); base.setRTS(False)
    time.sleep(2.5)  # base boot
    base.write(b"dfu 1\r\n")
    print("base: 'dfu 1' sent; waiting for bulb to join AP + get IP ...")
    deadline = time.time() + 30
    ip = None
    while time.time() < deadline:
        with open(LOG, "r", encoding="utf-8", errors="replace") as f:
            f.seek(start)
            new = f.read()
        m = re.findall(r"got IP (\d+\.\d+\.\d+\.\d+)", new)
        if m:
            ip = m[-1]
            break
        time.sleep(0.3)
    # Bulb is now on the AP (no longer sniffing), so quiet the base to keep its
    # channel-1 beacon injection from adding RF noise to the OTA transfer.
    base.write(b"stop\r\n")
    time.sleep(0.3)
    base.close()
    if ip:
        print("base: 'stop' sent (quiet channel for the push)")
    return ip


def main():
    ip = trigger_dfu_and_get_ip()
    if not ip:
        sys.exit("did not observe the bulb get an IP (is it powered / in range?)")
    print(f"bulb DFU IP: {ip}")

    data = open(IMG, "rb").read()
    good_sha = hashlib.sha256(data).digest()
    header = struct.pack("<IBI", MAGIC, 1, len(data)) + good_sha

    s = socket.create_connection((ip, PORT), timeout=30)

    if MODE == "half":
        s.sendall(header)
        half = len(data) // 2
        s.sendall(data[:half])
        print(f"[half] sent header + {half}/{len(data)} bytes, closing abruptly")
        s.close()

    elif MODE == "corrupt":
        bad = bytearray(data)
        mid = len(bad) // 2
        for i in range(mid, mid + 64):
            bad[i] ^= 0xFF  # flip 64 bytes; header still carries the ORIGINAL sha
        s.sendall(header)
        s.sendall(bytes(bad))
        print(f"[corrupt] sent full image with 64 corrupted bytes (sha won't match)")
        try:
            resp = s.recv(1)
            print("bulb status byte:", resp[0] if resp else "(none)")
        except Exception as e:
            print("recv:", e)
        s.close()

    elif MODE == "slow":
        s.sendall(header)
        delay = 0.5
        print(f"[slow] streaming {len(data)} bytes at 4KB / {delay*1000:.0f}ms "
              f"(~{len(data)/4096*delay:.0f}s). >>> RESET THE BULB PARTWAY THROUGH <<<")
        sent = 0
        try:
            while sent < len(data):
                chunk = data[sent:sent + 4096]
                s.sendall(chunk)
                sent += len(chunk)
                if sent % (4096 * 4) == 0:
                    print(f"  sent {sent}/{len(data)} bytes ({100*sent//len(data)}%)")
                time.sleep(delay)
            print("[slow] finished WITHOUT a reset; requesting status ...")
            try:
                print("bulb status byte:", s.recv(1))
            except Exception as e:
                print("recv:", e)
        except Exception as e:
            print(f"[slow] connection dropped at {sent}/{len(data)} bytes: {e}")
        s.close()

    else:
        sys.exit(f"unknown mode {MODE!r}")

    print("done; watch dfu-test.log for the bulb's reaction + reboot.")


if __name__ == "__main__":
    main()
