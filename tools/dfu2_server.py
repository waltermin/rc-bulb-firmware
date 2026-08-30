#!/usr/bin/env python3
"""DFU2 pull-based update server.

The base station broadcasts a `dfu2` request (id range + AP creds + this server's
IP/port + the target build id). Each bulb in range whose own build id differs
joins the AP and connects OUT to THIS server, which serves the firmware image and
tracks per-bulb progress and the overall rollout.

    python3 tools/dfu2_server.py build/bulb-firmware.bin --range 1-100 \
        --ssid "RC-Update" --pass "swordfish"

On start it prints the image's build id (the first 8 bytes of the ELF-SHA256 the
toolchain stamps into the app descriptor) and a ready-to-paste base-station
command, e.g.:

    dfu2 1 100 RC-Update swordfish 192.168.4.2 3333 deadbeef01020304

Wire protocol (little-endian), inverse of the old push DFU:
  * bulb -> server hello:  magic u32 | fmt u8 | bulb_id u8 | build_id[8]   (14 B)
  * server -> bulb header: magic u32 | version u8 | image_len u32 | sha256[32]
                           | build_id[8]                                   (49 B)
  * server -> bulb:        the raw image
  * bulb -> server:        one status byte (best-effort, before it reboots)
"""
import argparse
import asyncio
import hashlib
import json
import os
import socket
import struct
import sys
from datetime import datetime

# ---- wire constants (must match main/dfu2.c + main/config.h) -----------------
DFU_OTA_MAGIC = 0x686C4352      # "RClh" — transfer header magic
DFU2_HELLO_MAGIC = 0x32756C52   # "Rlu2" — bulb->server hello magic
DFU2_HELLO_FMT = 0x01
DFU2_HDR_VERSION = 0x01
BUILD_ID_LEN = 8
HELLO_LEN = 4 + 1 + 1 + BUILD_ID_LEN     # 14
DEFAULT_PORT = 3333
CHUNK = 4096

# esp_app_desc_t: magic(4) secure_version(4) reserv1(8) version[32]
# project_name[32] time[16] date[16] idf_ver[32] app_elf_sha256[32] reserv2[80]
APP_DESC_MAGIC = 0xABCD5432
APP_DESC_VERSION_OFFSET = 4 + 4 + 8          # 16: version[32] within the struct
APP_DESC_PROJECT_OFFSET = 16 + 32            # 48: project_name[32]
APP_DESC_LEN = 256

STATUS = {
    0: "OK — image accepted, bulb rebooting into new firmware",
    1: "bad header (magic/version/length rejected)",
    2: "esp_ota_begin failed",
    3: "receive error/timeout",
    4: "SHA-256 mismatch",
    5: "esp_ota_end validation failed",
    6: "esp_ota_set_boot_partition failed",
    7: "flashed image build id != advertised target",
    8: "bulb could not join AP / reach server",
}


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


def _printable_cstr(b):
    """A field is a plausible descriptor string if it is a non-empty, NUL-terminated
    run of printable ASCII. Used to reject spurious magic-word matches."""
    nul = b.find(b"\x00")
    if nul <= 0:
        return None
    s = b[:nul]
    if any(c < 0x20 or c > 0x7E for c in s):
        return None
    return s.decode("ascii")


def find_app_desc_version(data):
    """Locate the real esp_app_desc_t (the magic word can appear spuriously in
    code/rodata, so validate candidates by their printable version/project_name
    fields) and return its version string."""
    magic = struct.pack("<I", APP_DESC_MAGIC)
    off = 0
    while True:
        idx = data.find(magic, off)
        if idx < 0:
            break
        off = idx + 1
        d = data[idx:idx + APP_DESC_LEN]
        if len(d) < APP_DESC_LEN:
            continue
        version = _printable_cstr(d[APP_DESC_VERSION_OFFSET:APP_DESC_VERSION_OFFSET + 32])
        project = _printable_cstr(d[APP_DESC_PROJECT_OFFSET:APP_DESC_PROJECT_OFFSET + 32])
        if version and project:
            return version, project
    sys.exit("could not find a valid app descriptor in the image; is this a "
             "bulb-firmware .bin built by the ESP8266_RTOS_SDK?")


def extract_build_id(data):
    """Return the 8-byte build id = SHA-256(app version string)[:8]. The version
    string is set per build from version.txt (see build.sh); the ESP8266 toolchain
    does not stamp app_elf_sha256, so we hash the version instead — exactly what
    the bulb does at runtime (main/dfu2.c dfu2_own_build_id)."""
    version, project = find_app_desc_version(data)
    if not version:
        sys.exit("image has an empty version string; build.sh should write a "
                 "per-build version.txt.")
    build_id = hashlib.sha256(version.encode()).digest()[:BUILD_ID_LEN]
    extract_build_id.version = version   # stashed for the startup banner
    extract_build_id.project = project
    return build_id


def local_ip(target_hint="8.8.8.8"):
    """Best-effort primary local IPv4 (for the printed base-station command)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((target_hint, 80))
        return s.getsockname()[0]
    except OSError:
        return "0.0.0.0"
    finally:
        s.close()


class Server:
    def __init__(self, data, build_id, ids, args):
        self.data = data
        self.build_id = build_id
        self.sha = hashlib.sha256(data).digest()
        self.args = args
        self.header = (struct.pack("<IBI", DFU_OTA_MAGIC, DFU2_HDR_VERSION, len(data))
                       + self.sha + build_id)
        assert len(self.header) == 49
        self.state = {
            i: {"status": "waiting", "ip": None, "bytes": 0, "pct": 0,
                "attempts": 0, "cur_build": None, "started": None,
                "finished": None, "last_result": None}
            for i in ids
        }
        self.extra = {}   # ids seen outside the tracked range
        self.dirty = True

    # ---- state -------------------------------------------------------------
    def save(self):
        if not self.args.state:
            return
        tmp = self.args.state + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"updated": datetime.now().isoformat(timespec="seconds"),
                       "image": self.args.image,
                       "build_id": self.build_id.hex(),
                       "bulbs": {str(k): v for k, v in self.state.items()},
                       "extra": {str(k): v for k, v in self.extra.items()}},
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
              f"sending {c.get('sending', 0)}  connected {c.get('connected', 0)}  "
              f"failed {c.get('failed', 0)}  waiting {c.get('waiting', 0)}")
        print(f"  {'id':>4} {'status':<10} {'ip':<16} {'pct':>4}  last")
        for i, s in self.state.items():
            if s["status"] == "waiting" and not self.args.verbose_table:
                continue
            print(f"  {i:>4} {s['status']:<10} {s['ip'] or '-':<16} "
                  f"{s['pct']:>3}%  {s['last_result'] or ''}")
        waiting = [str(i) for i, s in self.state.items() if s["status"] == "waiting"]
        if waiting and not self.args.verbose_table:
            print(f"  waiting: {', '.join(waiting)}")
        failed = [str(i) for i, s in self.state.items() if s["status"] == "failed"]
        if failed:
            print(f"  FAILED (will retry if they re-broadcast): {', '.join(failed)}")
        if self.extra:
            print(f"  out-of-range bulbs seen: {', '.join(map(str, sorted(self.extra)))}")
        sys.stdout.flush()

    # ---- connection handler ------------------------------------------------
    async def handle(self, reader, writer):
        peer = writer.get_extra_info("peername")
        ip = peer[0] if peer else "?"
        bulb_id = None
        try:
            hello = await asyncio.wait_for(reader.readexactly(HELLO_LEN),
                                           timeout=self.args.timeout)
            magic, fmt, bulb_id = struct.unpack("<IBB", hello[:6])
            cur_build = hello[6:14]
            if magic != DFU2_HELLO_MAGIC or fmt != DFU2_HELLO_FMT:
                print(f"[{now()}] {ip}: bad hello (magic={magic:#x} fmt={fmt}); ignoring")
                return

            s = self.state.get(bulb_id)
            if s is None:
                s = self.extra.setdefault(bulb_id, {"status": "waiting", "ip": None,
                                                    "bytes": 0, "pct": 0, "attempts": 0,
                                                    "cur_build": None, "started": None,
                                                    "finished": None, "last_result": None})
            s["ip"] = ip
            s["cur_build"] = cur_build.hex()
            s["attempts"] += 1
            s["started"] = now()
            s["bytes"] = 0
            s["pct"] = 0
            s["status"] = "connected"
            self.dirty = True
            print(f"[{now()}] bulb {bulb_id} connected from {ip} "
                  f"(running build {cur_build.hex()}, attempt {s['attempts']})")

            if cur_build == self.build_id:
                # Shouldn't happen (bulb only connects on a mismatch) but guard.
                print(f"[{now()}] bulb {bulb_id} already on target build; still serving")

            # Send header + image, tracking progress.
            writer.write(self.header)
            await writer.drain()
            s["status"] = "sending"
            total = len(self.data)
            sent = 0
            while sent < total:
                writer.write(self.data[sent:sent + CHUNK])
                await writer.drain()
                sent += CHUNK
                s["bytes"] = min(sent, total)
                s["pct"] = 100 * s["bytes"] // total
                self.dirty = True

            # Best-effort final status byte before the bulb reboots.
            try:
                st = await asyncio.wait_for(reader.readexactly(1), timeout=self.args.timeout)
                code = st[0]
                s["last_result"] = f"status {code}: {STATUS.get(code, 'unknown')}"
                s["status"] = "ok" if code == 0 else "failed"
            except (asyncio.IncompleteReadError, asyncio.TimeoutError):
                # No status byte: the bulb likely rebooted right after the last
                # byte. A full send is a strong success signal, so treat as ok.
                s["last_result"] = "no status byte (bulb rebooted?) — assuming ok"
                s["status"] = "ok"
            s["finished"] = now()
            self.dirty = True
            print(f"[{now()}] bulb {bulb_id} -> {s['status']} ({s['last_result']})")
        except (asyncio.IncompleteReadError, asyncio.TimeoutError,
                ConnectionResetError, BrokenPipeError, OSError) as e:
            if bulb_id is not None:
                s = self.state.get(bulb_id) or self.extra.get(bulb_id)
                if s:
                    s["status"] = "failed"
                    s["last_result"] = f"transfer error: {e}"
                    s["finished"] = now()
            print(f"[{now()}] bulb {bulb_id if bulb_id is not None else ip} FAILED: {e}")
            self.dirty = True
        finally:
            try:
                writer.close()
            except OSError:
                pass

    # ---- run loop ----------------------------------------------------------
    async def run(self):
        server = await asyncio.start_server(self.handle, self.args.host, self.args.port)
        addrs = ", ".join(str(s.getsockname()) for s in server.sockets)
        print(f"[{now()}] DFU2 server listening on {addrs}")
        self.print_table()
        self.save()
        async with server:
            while True:
                await asyncio.sleep(1.0)
                if self.dirty:
                    self.dirty = False
                    self.save()
                    self.print_table()
                if all(s["status"] == "ok" for s in self.state.values()):
                    print(f"[{now()}] all {len(self.state)} bulbs updated")
                    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", help="firmware .bin to serve")
    ap.add_argument("--range", required=True,
                    help="bulb id range for overall progress, e.g. 1-100 or 1-10,20")
    ap.add_argument("--host", default="0.0.0.0", help="bind address")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--ssid", default="<ssid>", help="AP SSID (for the printed dfu2 command)")
    ap.add_argument("--pass", dest="passwd", default="<pass>",
                    help="AP password (for the printed dfu2 command)")
    ap.add_argument("--advertise-ip", default=None,
                    help="server IP to print in the dfu2 command (default: autodetect)")
    ap.add_argument("--state", default="dfu2_state.json",
                    help="JSON file mirrored on every change ('' to disable)")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="per-read timeout for hello / status byte")
    ap.add_argument("--verbose-table", action="store_true",
                    help="list every id in the table, including waiting ones")
    args = ap.parse_args()

    if not os.path.isfile(args.image):
        sys.exit(f"image not found: {args.image}")
    with open(args.image, "rb") as f:
        data = f.read()
    if not data:
        sys.exit("image is empty")

    build_id = extract_build_id(data)
    ids = parse_range(args.range)
    ip = args.advertise_ip or local_ip()
    lo, hi = ids[0], ids[-1]

    print(f"image  : {args.image} ({len(data)} bytes)")
    print(f"version: {getattr(extract_build_id, 'version', '?')!r} "
          f"(project {getattr(extract_build_id, 'project', '?')!r})")
    print(f"build  : {build_id.hex()}  (= sha256(version)[:8])")
    print(f"serving: {ip}:{args.port}  for bulb ids {lo}..{hi} ({len(ids)} bulbs)")
    print("\nPaste this on the base station (adjust the SSID/password if needed):\n")
    print(f"    dfu2 {lo} {hi} {args.ssid} {args.passwd} {ip} {args.port} {build_id.hex()}\n")

    srv = Server(data, build_id, ids, args)
    try:
        rc = asyncio.run(srv.run())
    except KeyboardInterrupt:
        srv.save()
        srv.print_table()
        rc = 130
    sys.exit(rc)


if __name__ == "__main__":
    main()
