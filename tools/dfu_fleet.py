#!/usr/bin/env python3
"""Fleet DFU pusher: watch DHCP for rc_light_dfu_<id> bulbs joining the AP and
push a firmware image to each one the moment it appears.

    python3 tools/dfu_fleet.py 1-25 build/bulb-firmware.bin -i Wi-Fi

How it works
  * tshark sniffs DHCP on the given interface. A bulb's DHCP Request is a
    broadcast that carries its hostname (option 12) and the IP it is taking
    (option 50 / ciaddr), so we learn id -> ip without router access.
  * Each new join spawns an asynchronous `push_dfu.py <ip> <image>`.
  * The bulb's DFU server accepts exactly ONE client per DFU boot
    (main/dfu.c), so a failed push is only retried when that id is seen
    joining again (power-cycle / re-trigger DFU). Connection-refused right
    after the join (server not listening yet) is retried quickly in-place.
  * State for every id in the range is kept in memory and mirrored to a JSON
    file (--state) after each change; a summary table prints on change and
    on exit.

Requires Wireshark's tshark on PATH (or the default Windows install path).
List interfaces with `tshark -D`.
"""
import argparse
import asyncio
import json
import os
import re
import shutil
import signal
import sys
import time
from datetime import datetime

HOST_PREFIX = "rc_light_dfu_"
HOST_RE = re.compile(re.escape(HOST_PREFIX) + r"(\d+)$")
HERE = os.path.dirname(os.path.abspath(__file__))
PUSH_DFU = os.path.join(HERE, "push_dfu.py")

# DHCP message types (option 53)
DHCP_REQUEST = "3"
DHCP_ACK = "5"

# tshark output columns, pipe separated
FIELDS = [
    "dhcp.option.dhcp",                  # message type
    "dhcp.option.hostname",
    "dhcp.option.requested_ip_address",  # option 50 (Request, SELECTING)
    "dhcp.ip.client",                    # ciaddr (Request, RENEWING)
    "dhcp.ip.your",                      # yiaddr (ACK)
    "dhcp.hw.mac_addr",
]

REFUSED_RE = re.compile(r"refused|10061|unreachable|10065|timed out", re.I)


def now():
    return datetime.now().strftime("%H:%M:%S")


def parse_range(spec):
    ids = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            ids.update(range(int(a), int(b) + 1))
        else:
            ids.add(int(part))
    if not ids:
        sys.exit("empty id range")
    return sorted(ids)


def find_tshark(explicit):
    if explicit:
        return explicit
    p = shutil.which("tshark")
    if p:
        return p
    for c in (r"C:\Program Files\Wireshark\tshark.exe",
              r"C:\Program Files (x86)\Wireshark\tshark.exe"):
        if os.path.exists(c):
            return c
    sys.exit("tshark not found; install Wireshark or pass --tshark PATH")


class Fleet:
    def __init__(self, ids, image, args):
        self.args = args
        self.image = image
        self.state = {
            i: {"status": "waiting", "ip": None, "mac": None, "attempts": 0,
                "last_seen": None, "last_result": None}
            for i in ids
        }
        self.mac_to_host = {}   # learned from Requests, used to resolve ACKs
        self.inflight = {}      # id -> asyncio.Task
        self.sem = asyncio.Semaphore(args.max_parallel)
        self.dirty = True

    # ---- state -------------------------------------------------------------
    def save(self):
        if not self.args.state:
            return
        tmp = self.args.state + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"updated": datetime.now().isoformat(timespec="seconds"),
                       "image": self.image,
                       "bulbs": {str(k): v for k, v in self.state.items()}},
                      f, indent=2)
        os.replace(tmp, self.args.state)

    def counts(self):
        c = {}
        for s in self.state.values():
            c[s["status"]] = c.get(s["status"], 0) + 1
        return c

    def print_table(self):
        c = self.counts()
        total = len(self.state)
        print(f"\n[{now()}] ok {c.get('ok', 0)}/{total}  "
              f"in_progress {c.get('in_progress', 0)}  "
              f"failed {c.get('failed', 0)}  waiting {c.get('waiting', 0)}")
        print(f"  {'id':>4} {'status':<12} {'ip':<16} {'mac':<18} {'att':>3}  last")
        for i, s in self.state.items():
            if s["status"] == "waiting" and not self.args.verbose_table:
                continue
            print(f"  {i:>4} {s['status']:<12} {s['ip'] or '-':<16} "
                  f"{s['mac'] or '-':<18} {s['attempts']:>3}  {s['last_result'] or ''}")
        waiting = [str(i) for i, s in self.state.items() if s["status"] == "waiting"]
        if waiting and not self.args.verbose_table:
            print(f"  waiting: {', '.join(waiting)}")
        failed = [str(i) for i, s in self.state.items() if s["status"] == "failed"]
        if failed:
            print(f"  FAILED (need re-trigger): {', '.join(failed)}")
        sys.stdout.flush()

    # ---- DHCP events -------------------------------------------------------
    def on_join(self, bulb_id, ip, mac):
        s = self.state.get(bulb_id)
        if s is None:
            if self.args.verbose:
                print(f"[{now()}] id {bulb_id} ({ip}) is outside the tracked range; ignoring")
            return
        s["last_seen"] = now()
        s["mac"] = mac or s["mac"]
        if s["status"] == "in_progress":
            return  # already pushing; the same join produces several packets
        if s["status"] == "ok":
            print(f"[{now()}] WARNING: id {bulb_id} re-entered DFU after a successful push "
                  f"(rollback? manual re-trigger?) -- pushing again")
        if s["attempts"] >= self.args.max_attempts:
            print(f"[{now()}] id {bulb_id} joined again but has hit --max-attempts; skipping")
            return
        s["ip"] = ip
        s["status"] = "in_progress"
        s["attempts"] += 1
        print(f"[{now()}] id {bulb_id} joined at {ip} ({mac}); pushing (attempt {s['attempts']})")
        self.inflight[bulb_id] = asyncio.create_task(self.push(bulb_id, ip))
        self.dirty = True

    async def push(self, bulb_id, ip):
        s = self.state[bulb_id]
        try:
            async with self.sem:
                # Bulb starts its listener right after GOT_IP; give it a beat.
                await asyncio.sleep(self.args.settle)
                deadline = time.monotonic() + self.args.connect_window
                while True:
                    rc, out = await self.run_push(ip)
                    if rc == 0:
                        s["status"] = "ok"
                        s["last_result"] = f"ok @ {now()}"
                        print(f"[{now()}] id {bulb_id} OK")
                        return
                    tail = (out.strip().splitlines() or ["(no output)"])[-1]
                    # Listener not up yet -> quick retry within the window.
                    if REFUSED_RE.search(out) and time.monotonic() < deadline:
                        await asyncio.sleep(1.0)
                        continue
                    s["status"] = "failed"
                    s["last_result"] = f"rc={rc}: {tail[:70]}"
                    print(f"[{now()}] id {bulb_id} FAILED: {tail}")
                    return
        except Exception as e:  # noqa: BLE001
            s["status"] = "failed"
            s["last_result"] = f"exception: {e}"
            print(f"[{now()}] id {bulb_id} FAILED: {e}")
        finally:
            self.inflight.pop(bulb_id, None)
            self.dirty = True

    async def run_push(self, ip):
        cmd = [sys.executable, PUSH_DFU, ip, self.image,
               "--timeout", str(self.args.push_timeout)]
        if self.args.port:
            cmd += ["--port", str(self.args.port)]
        proc = await asyncio.create_subprocess_exec(
            *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        out_b, _ = await proc.communicate()
        out = out_b.decode(errors="replace")
        if self.args.verbose:
            for line in out.splitlines():
                print(f"    [{ip}] {line}")
        return proc.returncode, out

    # ---- sniffer -----------------------------------------------------------
    def handle_line(self, line):
        cols = line.rstrip("\r\n").split("|")
        if len(cols) < len(FIELDS):
            return
        mtype, host, req_ip, ciaddr, yiaddr, mac = cols[:len(FIELDS)]
        # tshark may emit comma-joined values for repeated fields
        host = host.split(",")[0].strip()
        mac = mac.split(",")[0].strip().lower()

        if host:
            m = HOST_RE.match(host)
            if not m:
                return
            bulb_id = int(m.group(1))
            if mac:
                self.mac_to_host[mac] = bulb_id
        elif mac in self.mac_to_host:
            bulb_id = self.mac_to_host[mac]
        else:
            return

        ip = None
        if mtype == DHCP_REQUEST:
            ip = req_ip or (ciaddr if ciaddr and ciaddr != "0.0.0.0" else None)
        elif mtype == DHCP_ACK:
            ip = yiaddr if yiaddr and yiaddr != "0.0.0.0" else None
        if ip:
            self.on_join(bulb_id, ip, mac)

    async def sniff(self, tshark):
        cmd = [tshark, "-i", self.args.interface, "-l", "-n",
               "-f", "udp port 67 or udp port 68",
               "-Y", f'dhcp.option.hostname contains "{HOST_PREFIX}" or dhcp.option.dhcp == {DHCP_ACK}',
               "-T", "fields", "-E", "separator=|"]
        for f in FIELDS:
            cmd += ["-e", f]
        print(f"[{now()}] sniffing DHCP on '{self.args.interface}' via {tshark}")
        proc = await asyncio.create_subprocess_exec(
            *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        self.sniffer = proc

        async def drain_stderr():
            async for raw in proc.stderr:
                line = raw.decode(errors="replace").rstrip()
                if line and "Capturing on" not in line and not line.isdigit():
                    print(f"[tshark] {line}")
        asyncio.create_task(drain_stderr())

        async for raw in proc.stdout:
            self.handle_line(raw.decode(errors="replace"))
        rc = await proc.wait()
        raise RuntimeError(f"tshark exited with code {rc} (bad interface? run `tshark -D`; "
                           f"on Linux you may need sudo/CAP_NET_RAW)")

    # ---- main loop ---------------------------------------------------------
    async def run(self, tshark):
        sniff_task = asyncio.create_task(self.sniff(tshark))
        self.print_table()
        self.save()
        start = time.monotonic()
        try:
            while True:
                await asyncio.sleep(1.0)
                if self.dirty:
                    self.dirty = False
                    self.save()
                    self.print_table()
                if sniff_task.done():
                    sniff_task.result()  # raises
                if all(s["status"] == "ok" for s in self.state.values()):
                    print(f"[{now()}] all {len(self.state)} bulbs updated")
                    return 0
                if self.args.deadline and time.monotonic() - start > self.args.deadline:
                    print(f"[{now()}] deadline reached")
                    return 1
        finally:
            sniff_task.cancel()
            if getattr(self, "sniffer", None) and self.sniffer.returncode is None:
                self.sniffer.kill()
            if self.inflight:
                print(f"[{now()}] waiting for {len(self.inflight)} in-flight push(es)...")
                await asyncio.gather(*self.inflight.values(), return_exceptions=True)
            self.save()
            self.print_table()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", help="bulb id range, e.g. 1-25 or 1-10,14,20-25")
    ap.add_argument("image", help="firmware .bin to push")
    ap.add_argument("-i", "--interface", default="Wi-Fi",
                    help="capture interface name/number (tshark -D)")
    ap.add_argument("--tshark", help="path to tshark")
    ap.add_argument("-p", "--port", type=int, help="DFU TCP port (default push_dfu.py's 3333)")
    ap.add_argument("--state", default="dfu_fleet_state.json",
                    help="JSON file mirrored on every change ('' to disable)")
    ap.add_argument("--max-parallel", type=int, default=4,
                    help="concurrent pushes (Wi-Fi bandwidth bound)")
    ap.add_argument("--max-attempts", type=int, default=3,
                    help="pushes per id across re-joins")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds to wait after DHCP before connecting")
    ap.add_argument("--connect-window", type=float, default=20.0,
                    help="seconds to keep retrying 'connection refused' after a join")
    ap.add_argument("--push-timeout", type=float, default=60.0,
                    help="socket timeout passed to push_dfu.py")
    ap.add_argument("--deadline", type=float, default=0,
                    help="give up after this many seconds (0 = run until Ctrl-C)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="echo push_dfu.py output and out-of-range joins")
    ap.add_argument("--verbose-table", action="store_true",
                    help="list every id in the table, including waiting ones")
    args = ap.parse_args()

    if not os.path.isfile(args.image):
        sys.exit(f"image not found: {args.image}")
    if not os.path.isfile(PUSH_DFU):
        sys.exit(f"push_dfu.py not found next to this script: {PUSH_DFU}")
    ids = parse_range(args.ids)
    tshark = find_tshark(args.tshark)
    fleet = Fleet(ids, os.path.abspath(args.image), args)

    try:
        rc = asyncio.run(fleet.run(tshark))
    except KeyboardInterrupt:
        rc = 130
    sys.exit(rc)


if __name__ == "__main__":
    main()
