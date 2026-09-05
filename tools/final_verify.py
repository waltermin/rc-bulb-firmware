#!/usr/bin/env python3
"""Final end-to-end verification of the RC switch on OpenBeken:
  1. broadcast RelaysUpdate id5=ON  -> relay (P9/SW) should be HIGH
  2. broadcast RelaysUpdate id5=OFF -> relay should go LOW
  3. stop broadcasting               -> relay reverts to default (OFF) after ~15s
"""
import time, serial
from serial.tools import list_ports
MY="5C94136993"
port=[p.device for p in list_ports.comports() if p.serial_number and MY in p.serial_number][0]
def cmd(s,c,w=0.4):
    s.reset_input_buffer(); s.write(c.encode()+b"\n"); time.sleep(w)
    return s.read(256).decode(errors="replace")
def relay(s):
    s.reset_input_buffer(); s.write(b"SW?\n")
    try: return int(s.read_until(b"\n",16).strip())
    except: return None
def frac_high(s, secs):
    hi=n=0; t=time.time()
    while time.time()-t<secs:
        v=relay(s)
        if v is not None: n+=1; hi+=(v==1)
        time.sleep(0.25)
    return hi, n

s=serial.Serial(port,115200,timeout=0.2); time.sleep(2)
cmd(s,"RELAYS 5:1"); cmd(s,"AUTO 300"); cmd(s,"CEN PULSE 100")
print("reset + broadcasting id5=ON; waiting 8s for boot+monitor"); time.sleep(8)
hi,n=frac_high(s,5); print(f"[1] id5=ON  -> relay HIGH {hi}/{n}  {'PASS' if hi>n*0.7 else 'FAIL'}")
cmd(s,"RELAYS 5:0"); time.sleep(2)
hi,n=frac_high(s,5); print(f"[2] id5=OFF -> relay HIGH {hi}/{n}  {'PASS' if hi<n*0.3 else 'FAIL'}")
cmd(s,"RELAYS 5:1"); time.sleep(2)
hi,n=frac_high(s,4); print(f"[3] id5=ON again -> relay HIGH {hi}/{n}  {'PASS' if hi>n*0.7 else 'FAIL'}")
cmd(s,"AUTO OFF"); print("[4] stopped broadcasting; waiting 18s for fallback-to-default(OFF)"); time.sleep(18)
hi,n=frac_high(s,4); print(f"[4] silence -> relay HIGH {hi}/{n}  {'PASS(reverted OFF)' if hi<n*0.3 else 'FAIL'}")
s.close()
