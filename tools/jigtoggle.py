#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial"]
# ///
"""Web control page for lights driven over the air by the BKJIG2 jig.

  uv run tools/jigtoggle.py [--bulbs 0-63] [--switches 64-115] [--channel 4] [--port 8765]
  then open http://localhost:8765

Two independent groups, each held continuously by the jig until changed:
  bulbs     on/off, colour (hue/saturation wheel) or white (colour-temperature
            slider over the warm/cold channels), with a shared brightness
  switches  relay on/off

Changes are latest-wins: the page posts the desired state, a background worker
pushes only the newest one to the jig, so dragging the wheel never builds a queue.
"""
import argparse
import colorsys
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import bkjig2  # noqa: E402

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Jig Lights</title>
<style>
:root { --bg:#f6f5f2; --card:#fff; --ink:#1d1d1f; --muted:#6b6b70; --line:#e2e0da;
        --accent:#2f6fde; --err:#c0392b; --track:#e9e7e1; }
@media (prefers-color-scheme: dark) { :root { --bg:#141416; --card:#1d1d21; --ink:#f2f2f4;
        --muted:#9a9aa2; --line:#2e2e34; --accent:#5b8ff0; --err:#e5675a; --track:#2a2a30; } }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--ink);
       font:16px/1.4 -apple-system, system-ui, sans-serif; display:flex; justify-content:center; }
main { width:100%; max-width:520px; padding:28px 16px 40px; display:grid; gap:16px; }
h1 { font-size:20px; margin:0; }
.sub { color:var(--muted); font-size:14px; margin:-8px 0 0; }
.card { background:var(--card); border:1px solid var(--line); border-radius:16px; padding:16px; display:grid; gap:14px; }
.head { display:flex; align-items:center; justify-content:space-between; gap:12px; }
.head h2 { font-size:17px; margin:0; }
.head .ids { color:var(--muted); font-size:13px; font-weight:400; margin-left:6px; }
.switch { position:relative; width:52px; height:30px; flex:none; }
.switch input { opacity:0; width:0; height:0; }
.switch span { position:absolute; inset:0; background:var(--track); border-radius:30px; transition:.15s; cursor:pointer; }
.switch span::before { content:""; position:absolute; left:3px; top:3px; width:24px; height:24px;
                       background:#fff; border-radius:50%; transition:.15s; box-shadow:0 1px 3px #0003; }
.switch input:checked + span { background:var(--accent); }
.switch input:checked + span::before { transform:translateX(22px); }
.body { display:grid; gap:14px; }
.tabs { display:grid; grid-template-columns:1fr 1fr; background:var(--track); border-radius:10px; padding:3px; }
.tabs button { font:inherit; font-size:14px; border:0; background:none; color:var(--muted); padding:7px; border-radius:8px; cursor:pointer; }
.tabs button.on { background:var(--card); color:var(--ink); box-shadow:0 1px 2px #0002; }
.wheelwrap { display:flex; justify-content:center; }
.wheel { position:relative; width:min(260px, 72vw); aspect-ratio:1; border-radius:50%; touch-action:none; cursor:crosshair;
         background:radial-gradient(circle closest-side, #fff, #fff0),
                    conic-gradient(from 90deg, red, magenta, blue, cyan, lime, yellow, red); }
.dot { position:absolute; width:22px; height:22px; margin:-11px 0 0 -11px; border-radius:50%;
       border:3px solid #fff; box-shadow:0 0 0 1px #0006, 0 1px 4px #0005; pointer-events:none; }
label.row { display:grid; gap:6px; font-size:14px; color:var(--muted); }
label.row .v { float:right; color:var(--ink); font-variant-numeric:tabular-nums; }
input[type=range] { width:100%; accent-color:var(--accent); }
#temp { appearance:none; -webkit-appearance:none; height:10px; border-radius:6px;
        background:linear-gradient(90deg, #ffb35c, #fff3e0, #cfe3ff); }
#temp::-webkit-slider-thumb { -webkit-appearance:none; width:22px; height:22px; border-radius:50%;
        background:#fff; border:2px solid var(--ink); }
#temp::-moz-range-thumb { width:22px; height:22px; border-radius:50%; background:#fff; border:2px solid var(--ink); }
.swatch { height:10px; border-radius:6px; border:1px solid var(--line); }
.dim { opacity:.45; }
.status { font-size:13px; color:var(--muted); min-height:1.3em; }
.status.err { color:var(--err); }
[hidden] { display:none !important; }
</style></head><body><main>
<h1>Jig lights</h1>
<p class="sub">Channel __CH__ &middot; held by the jig until changed</p>

<section class="card">
  <div class="head"><h2>Bulbs<span class="ids">__BU__</span></h2>
    <label class="switch"><input type="checkbox" id="b-on"><span></span></label></div>
  <div class="body" id="b-body">
    <div class="tabs"><button data-mode="color">Colour</button><button data-mode="white">White</button></div>
    <div id="color-pane" class="wheelwrap"><div class="wheel" id="wheel"><div class="dot" id="dot"></div></div></div>
    <label class="row" id="white-pane">Colour temperature <span class="v" id="temp-v"></span>
      <input type="range" id="temp" min="0" max="1000" value="0"></label>
    <label class="row">Brightness <span class="v" id="bri-v"></span>
      <input type="range" id="bri" min="1" max="100" value="100"></label>
    <div class="swatch" id="swatch"></div>
  </div>
</section>

<section class="card">
  <div class="head"><h2>Switches<span class="ids">__SW__</span></h2>
    <label class="switch"><input type="checkbox" id="s-on"><span></span></label></div>
</section>

<div class="status" id="status"></div>
<script>
const $ = id => document.getElementById(id);
let st = null;   // last state from the server
let b = {};      // local bulb controls: on, mode, h, s, v, t
const status = $("status");

const kelvin = t => Math.round(2700 + t * (6500 - 2700));
function showStatus(s) {
  status.className = "status" + (s.error ? " err" : "");
  status.textContent = s.error ? "Error: " + s.error : (s.applied_at ? "Applied " + s.applied_at : "Not changed from this page yet");
}
function renderBulbs() {
  $("b-on").checked = b.on;
  $("b-body").classList.toggle("dim", !b.on);
  document.querySelectorAll(".tabs button").forEach(x => x.classList.toggle("on", x.dataset.mode === b.mode));
  $("color-pane").hidden = b.mode !== "color";
  $("white-pane").hidden = b.mode !== "white";
  const r = $("wheel").getBoundingClientRect().width / 2, ang = b.h * 2 * Math.PI;
  $("dot").style.left = (r + Math.cos(ang) * b.s * r) + "px";
  $("dot").style.top = (r - Math.sin(ang) * b.s * r) + "px";
  $("dot").style.background = `hsl(${b.h * 360} 100% ${100 - b.s * 50}%)`;
  $("temp").value = Math.round(b.t * 1000);
  $("temp-v").textContent = kelvin(b.t) + " K";
  $("bri").value = Math.round(b.v * 100);
  $("bri-v").textContent = Math.round(b.v * 100) + "%";
  const ww = Math.min(1, 2 * (1 - b.t)), cw = Math.min(1, 2 * b.t);
  $("swatch").style.background = b.mode === "color"
    ? `hsl(${b.h * 360} 100% ${100 - b.s * 50}%)`
    : `linear-gradient(90deg, rgba(255,179,92,${ww}), rgba(207,227,255,${cw}))`;
}
let inflight = false, pending = false;
async function post(path, body) {
  try {
    const r = await fetch(path, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
    st = await r.json(); showStatus(st);
  } catch (e) { showStatus({error: "server not reachable"}); }
}
function sendBulbs() {   // one request in flight; the newest change rides the next one
  if (inflight) { pending = true; return; }
  inflight = true;
  post("/bulbs", b).finally(() => { inflight = false; if (pending) { pending = false; sendBulbs(); } });
}
const wheel = $("wheel");
function wheelAt(e) {
  const w = wheel.getBoundingClientRect(), r = w.width / 2;
  const x = e.clientX - w.left - r, y = r - (e.clientY - w.top);
  b.h = ((Math.atan2(y, x) / (2 * Math.PI)) + 1) % 1;
  b.s = Math.min(1, Math.hypot(x, y) / r);
  b.mode = "color"; b.on = true; renderBulbs(); sendBulbs();
}
wheel.addEventListener("pointerdown", e => { wheel.setPointerCapture(e.pointerId); wheelAt(e); });
wheel.addEventListener("pointermove", e => { if (wheel.hasPointerCapture(e.pointerId)) wheelAt(e); });
$("temp").addEventListener("input", e => { b.t = e.target.value / 1000; b.mode = "white"; b.on = true; renderBulbs(); sendBulbs(); });
$("bri").addEventListener("input", e => { b.v = e.target.value / 100; b.on = true; renderBulbs(); sendBulbs(); });
document.querySelectorAll(".tabs button").forEach(x => x.addEventListener("click", () => {
  b.mode = x.dataset.mode; b.on = true; renderBulbs(); sendBulbs();
}));
$("b-on").addEventListener("change", e => { b.on = e.target.checked; renderBulbs(); sendBulbs(); });
$("s-on").addEventListener("change", e => post("/switches", {on: e.target.checked}));
window.addEventListener("resize", renderBulbs);
(async () => {
  try { st = await (await fetch("/state")).json(); } catch (e) { showStatus({error: "server not reachable"}); return; }
  b = Object.assign({}, st.bulbs); delete b.rgbcw;
  $("s-on").checked = st.switches.on;
  renderBulbs(); showStatus(st);
})();
</script></main></body></html>"""


def bulb_rgbcw(b):
    """Desired bulb state -> (r, g, b, ww, cw) bytes."""
    if not b["on"]:
        return (0, 0, 0, 0, 0)
    v = max(0.0, min(1.0, b["v"]))
    if b["mode"] == "color":
        r, g, bl = colorsys.hsv_to_rgb(b["h"] % 1.0, max(0.0, min(1.0, b["s"])), v)
        return (round(r * 255), round(g * 255), round(bl * 255), 0, 0)
    t = max(0.0, min(1.0, b["t"]))   # 0 = warmest, 1 = coolest
    ww = min(1.0, 2 * (1 - t)) * v   # full warm at the warm end, both full mid-way
    cw = min(1.0, 2 * t) * v
    return (0, 0, 0, round(ww * 255), round(cw * 255))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bulbs", default="0-63")
    ap.add_argument("--switches", default="64-115")
    ap.add_argument("--channel", type=int, default=4)
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()

    bulb_ids = sorted(bkjig2.parse_ids(a.bulbs))
    switch_ids = bkjig2.parse_ids(a.switches)
    chunks = [bulb_ids[i:i + bkjig2.BULB_ENTRIES_PER_PACKET]
              for i in range(0, len(bulb_ids), bkjig2.BULB_ENTRIES_PER_PACKET)]
    if len(chunks) > bkjig2.RAW_SLOTS:
        sys.exit(f"at most {bkjig2.RAW_SLOTS * bkjig2.BULB_ENTRIES_PER_PACKET} bulbs")

    # Starting assumption: switches off, bulbs at their own default (full warm),
    # which is what un-held devices fall back to. Nothing is sent until a change.
    desired = {"bulbs": {"on": True, "mode": "white", "h": 0.08, "s": 1.0, "v": 1.0, "t": 0.0},
               "switches": {"on": False}}
    lock = threading.Lock()
    wake = threading.Event()
    info = {"applied_at": None, "error": None}
    applied = {"bulbs": None, "switches": None}

    def apply_once():
        with lock:
            want_b = bulb_rgbcw(desired["bulbs"])
            want_s = desired["switches"]["on"]
        port = bkjig2.find_port()
        if want_s != applied["switches"]:
            bits = bkjig2.bitfield_hex(switch_ids if want_s else set())
            bkjig2.command(port, f"HOLD RELAY {bits} {a.channel}", ["HOLD RELAY OK", "ERR"], 3)
            applied["switches"] = want_s
        if want_b != applied["bulbs"]:
            if applied["bulbs"] is None:   # first bulb write: drop any older raw holds
                bkjig2.command(port, "HOLD RAW OFF", ["HOLD RAW OFF OK"], 3)
            for slot, chunk in enumerate(chunks):
                body = bytes([0x03, len(chunk)]) + b"".join(bytes([i, *want_b]) for i in chunk)
                bkjig2.command(port, f"HOLD RAW {slot} {body.hex()} {a.channel}", ["HOLD RAW", "ERR"], 3)
            applied["bulbs"] = want_b

    def worker():
        while True:
            wake.wait()
            wake.clear()
            try:
                apply_once()
                info.update(applied_at=time.strftime("%H:%M:%S"), error=None)
            except (Exception, SystemExit) as e:
                info["error"] = str(e) or type(e).__name__
                applied["bulbs"] = applied["switches"] = None   # resend everything next time

    threading.Thread(target=worker, daemon=True).start()

    page = (PAGE.replace("__BU__", a.bulbs).replace("__SW__", a.switches)
                .replace("__CH__", str(a.channel)).encode())

    def snapshot():
        with lock:
            bulbs = dict(desired["bulbs"], rgbcw=list(bulb_rgbcw(desired["bulbs"])))
            return {"bulbs": bulbs, "switches": dict(desired["switches"]), **info}

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self._send(200, page, "text/html; charset=utf-8")
            elif self.path == "/state":
                self._send(200, json.dumps(snapshot()).encode(), "application/json")
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            group = self.path.strip("/")
            if group not in ("bulbs", "switches"):
                return self._send(404, b"not found", "text/plain")
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            except ValueError:
                return self._send(400, b"bad json", "text/plain")
            with lock:
                cur = desired[group]
                for k in cur:
                    if k not in body:
                        continue
                    if k == "on":
                        cur[k] = bool(body[k])
                    elif k == "mode":
                        cur[k] = body[k] if body[k] in ("color", "white") else cur[k]
                    else:
                        cur[k] = float(body[k])
            wake.set()
            self._send(200, json.dumps(snapshot()).encode(), "application/json")

        def log_message(self, fmt, *args):
            pass

    print(f"jig lights: http://localhost:{a.port}  (bulbs {a.bulbs}, switches {a.switches}, channel {a.channel})")
    ThreadingHTTPServer(("127.0.0.1", a.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
