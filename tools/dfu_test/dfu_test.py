#!/usr/bin/env python3
"""End-to-end DFU2 update test (time-boxed reads; passthrough only while watching).

Running rc_bare (relay default OFF) -> OTA to rc_v2 (relay default ON). Success =
relay defaults HIGH after the update with no broadcast.
Jig = softAP "RC-Update" + TCP update server + 0x05 broadcaster + UART passthrough.
"""
import hashlib, sys, time
import serial
from serial.tools import list_ports

MY = "5C94136993"
RBL = r"C:\Users\rose\rc-switch-firmware\output\rc_v2.rbl"


def find_port():
    for p in list_ports.comports():
        if p.serial_number and MY in p.serial_number:
            return p.device
    raise SystemExit("jig not found")


def read_for(s, secs):
    """Read everything for a fixed duration (works even under log flooding)."""
    out = b""; t = time.time()
    while time.time() - t < secs:
        d = s.read(1024)
        if d:
            out += d
    return out.decode(errors="replace")


def cmd(s, c, secs=1.5):
    s.reset_input_buffer(); s.write(c.encode() + b"\n")
    return read_for(s, secs)


def relay(s):
    s.reset_input_buffer(); s.write(b"SW?\n")
    r = read_for(s, 0.6)
    for tok in r.split():
        if tok in ("0", "1"):
            return int(tok)
    return None


rbl = open(RBL, "rb").read()
sha = hashlib.sha256(rbl).hexdigest()
buildid = hashlib.sha256(b"rc_v2").hexdigest()[:16]
print(f"RBL {len(rbl)} bytes sha={sha[:16]}.. buildid={buildid}", flush=True)

s = serial.Serial(find_port(), 115200, timeout=0.2)
time.sleep(2)
print("ping:", cmd(s, "PING").strip(), flush=True)
cmd(s, "PASS OFF")                                  # clean command phase
print("AP:", cmd(s, "DFUAP", 3).strip(), flush=True)
print("load prompt:", cmd(s, f"DFULOAD {len(rbl)} {sha} {buildid}", 1).strip(), flush=True)
for i in range(0, len(rbl), 4096):
    s.write(rbl[i:i+4096]); s.flush()
print("load result:", read_for(s, 4).strip(), flush=True)

print("resetting switch; 9s boot", flush=True)
cmd(s, "CEN PULSE 100", 1); time.sleep(9)
print(f"baseline relay (rc_bare, expect 0): {relay(s)}", flush=True)

cmd(s, "PASS ON", 0.5)                              # now watch switch + jig logs
print("--- broadcasting DFU2 request repeatedly + watching (120s) ---", flush=True)
t = time.time(); saw = set(); last_bc = 0; triggered = False
while time.time() - t < 120:
    # rebroadcast the 0x05 every ~2s until the switch enters DFU (like a real
    # base station), then stop (the switch left monitor mode anyway).
    if not triggered and time.time() - last_bc > 2:
        cmd(s, "DFU2BC 5", 0.3); last_bc = time.time()
    out = read_for(s, 2)
    if "entering" in out or "client connected" in out:
        triggered = True
    for ln in out.splitlines():
        ls = ln.strip()
        if any(k in ls for k in ("DFU2:", "dfu2", "receiving", "got ip", "status=",
                                 "sent ", "os_addr", "start_type", "poweron")):
            print("  >", ls[:120], flush=True)
    if "status=0" in out or "status=3" in out:
        break

cmd(s, "PASS OFF", 0.5)
# The bootloader now decompresses the RBL and rewrites the ~448KB app slot.
# This takes a while and MUST NOT be interrupted (no CEN pulses) — rt_ota
# restarts the swap on any reset until it completes. Just wait.
print("waiting 120s for rt_ota swap + rc_v2 boot (NO reset)...", flush=True)
time.sleep(120)
vals = []
for _ in range(10):
    vals.append(relay(s)); time.sleep(0.4)
hi = sum(1 for v in vals if v == 1)
print(f"post-update relay HIGH {hi}/{len(vals)} -> "
      f"{'PASS: rc_v2 running (relay default ON)' if hi > 6 else 'not swapped (still rc_bare or failed)'}", flush=True)
s.close()
