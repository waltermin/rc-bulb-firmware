# PWM Engine Rewrite — Handoff

Status as of build `rc-fead6f3ea13f`. This documents a large, multi-stage effort to
replace the bulb's PWM driver and then chase down flicker. **The core feature work is
done and solid; one residual flicker issue remains, with a clear decision point.**

---

## 1. What the task was

Replace the SDK's built-in software PWM (`driver/pwm.h`) with a ground-up engine where
**each channel has its own duty, phase, and PWM period**. Periods are powers of two (in
ticks) so all channels are harmonics that stack into one repeatable edge table. Then a
`0x06 RawLightUpdate` protocol packet was added, and a long debugging effort to make the
PWM **flicker-free** followed.

---

## 2. Architecture that shipped (all working)

- **`main/pwm_schedule.{c,h}`** — PURE, host-testable schedule compiler. `pwm_compile()`
  turns per-channel `{duty, phase, period_log2}` into one sorted edge table. Edges are
  packed `uint32_t` = `(24-bit tick offset << 8) | 8-bit channel-state`. Handles full-on,
  rounds-to-off, phase wrap, merging simultaneous edges, all-off. No SDK deps. Unit tests
  in `test/test_pwm_schedule.c` (pass).
- **`main/pwm_output.{c,h}`** — the engine. Keeps the old duty math (clamp → gamma/
  perceptual curve → max-power cap) verbatim. Builds `pwm_chan_req_t[]`, compiles a
  schedule, double-buffers it, and an ISR walks the edge table. Two 256-entry LUTs map
  the state byte → GPIO set/clear words (one store each). `pwm_output_set_raw()` is the
  raw path (clamp-only, per-channel periods). Dedup: identical `reqs` skip recompile.
- **`main/mono_clock.{c,h}`** — the monotonic-clock shim (see §5). Runtime-installs a copy
  of the RTOS tick handler.
- **Ticks:** one tick = 200 ns (`PWM_TICK_NS`). `n` range `PWM_PERIOD_LOG2_MIN..MAX` =
  10..16. Default period `n=12` (~1.22 kHz). `PWM_MAX_CHANNELS 5`.
- **Timer:** the engine rides the **Wi-Fi WDEV/TSF0 timer** (same one the SDK PWM uses),
  registered via the private `wDev_MacTimSetFunc`. Its interrupt is **NMI level** — this
  is why it's smoother than a level-1 timer. Runs `pwm_output_start_after_radio()` after
  `esp_wifi_start()` because the TSF timer needs the radio. Two-phase init: `pwm_output_init()`
  shows a static default color, `pwm_output_start_after_radio()` begins real PWM.
- **Protocol `0x06 RawLightUpdate`** — done, tested, flashed. Wire format in `spec.txt`
  and `protocol.h`: `bulb_id`, 5×f32 duties, 5×u8 periods (order **r,g,b,ww,cw**, matching
  colors — the user fixed an earlier cw/ww swap). Parser `parse_raw_light_update` sets
  `has_raw_entry`; sniffer → `controller_notify_raw_entry` → `pwm_output_set_raw`.
  `tools/send_update.py --raw id:r,g,b,ww,cw:pr,pg,pb,pww,pcw`. Host tests pass (92/103).

### Debug scaffolding currently ENABLED (strip before shipping)
- `PWM_DEBUG_DUMP 1` (config.h) — `pwm_compile()` prints the full schedule each build.
- `PWM_PROFILE 1` (config.h) — the ISR bumps DRAM counters; `pwm_profile_task` prints a
  `[pwmprof]` line/sec: `inv` (ISR invocations), `fgap` (max inter-ISR gap), `bkwd`
  (count/max of CCOUNT going backward), `ahead` (min/max of `s_target - now`), `spin`
  (max spin µs), `reanc`, `scap`, `tsf`. Extremely useful — keep until flicker is closed.

---

## 3. How the ISR works (current design)

FRC1 was abandoned (see §4). Now on the WDEV/TSF0 NMI timer:

1. **Arm** the TSF compare to fire ~`PWM_AHEAD_TICKS` (40 ticks = 8 µs) *before* the next
   edge. Arm delay is **relative** and schedule-derived (`gap - AHEAD`), so it's always
   bounded (never 0, never huge). `wdev_arm()` zeroes the TSF and sets a relative compare
   (replicates the SDK PWM's exact register dance).
2. **Spin** on `mono_ccount()` to the edge's exact target `s_target`, then write GPIO.
   The spin is bounded by an **iteration count** (`PWM_SPIN_MAX_ITERS`), NOT a clock delta
   — critical, see §5.
3. **Re-anchor**: `s_target` advances by exact tick gaps (locks frequency), but snaps to
   `now` if an edge lands late by > `ahead` and < `PWM_REANCHOR_MAX_TICKS` (rejects
   implausible ~10 ms jumps from the clock race).
4. Double-buffer swap happens at the cycle wrap when `s_update_pending`.

`s_target`, spin, and re-anchor all read **`mono_ccount()`** (from `mono_clock.h`), NOT
raw `soc_get_ccount()`.

---

## 4. The debugging journey (why things are the way they are)

This is the important context — many "obvious" approaches were tried and failed for
concrete reasons. Don't re-tread these:

1. **Stack overflow.** `pwm_compile`'s scratch `toggles[]` (~4 KB) was on the stack →
   overflowed the main/controller task stacks → silent corruption → `LoadStoreError` in
   Wi-Fi init. Fixed: made it `static` (single producer, never concurrent). **Watch
   embedded stack budgets.**
2. **FRC1 timer + relative arming = jitter.** First timer choice was FRC1 (`hw_timer`,
   level-1 interrupt). Edges landed at `scheduled + interrupt_latency`; latency varied →
   duty jitter → candle flicker, worse at high freq. FRC1 is level-1, so it's masked by
   `portENTER_CRITICAL` and delayed by other ISRs — unavoidable jitter.
3. **CCOUNT absolute timebase + FRC1 → strobe/crash.** Introduced an absolute CCOUNT
   `s_target` with fire-early + spin-to-exact. Bugs found & fixed along the way:
   `hw_timer_disarm()` zeroes clkdiv (must set clkdiv AFTER); the CPU is **160 MHz not
   80** (SDK default `CONFIG_ESP8266_DEFAULT_CPU_FREQ_160`) so cycles/tick is 32 not 16 —
   detected at runtime via `esp_clk_cpu_freq()` → `s_cyc_shift` (5 at 160 MHz). LX106 has
   **no hardware divide**, so the ISR uses shifts only (a libgcc divide is flash-resident,
   unsafe in the OTA-safe ISR).
4. **Switched FRC1 → WDEV/TSF0 (NMI).** FRC1's level-1 latency can't be beaten from level
   1. The SDK PWM is glassy because it uses the WDEV timer at **NMI level**. Switched to
   it (same `wDev_MacTimSetFunc` mechanism, proven to coexist with Wi-Fi, documented
   interference = packet loss, which the user accepts — 50 Hz updates, loss-tolerant
   protocol). This removed most flicker but exposed the next bug.
5. **CCOUNT is not free-running (the big one).** The profiler showed `bkwd=100/10001` —
   CCOUNT steps **backward ~10 ms, 100×/sec** = once per RTOS tick. `soc_get_ccount()` on
   this SDK sawtooths 0..(one tick) because `xPortSysTickHandle`
   (`freertos/port/esp8266/port.c`) **zeroes CCOUNT every tick** (to bound overflow /
   calibrate time across flash ops). So CCOUNT is unusable as an absolute clock. `s_target`
   ran away, spins maxed, everything broke.
6. **Monotonic clock via the SDK's own accumulator.** The tick handler already does
   `g_esp_os_cpu_clk += ccount; soc_set_ccount(0);` — so `g_esp_os_cpu_clk + soc_get_ccount()`
   is a true monotonic cycle count. Implemented `mono_ccount()`. To avoid forking the SDK,
   `mono_clock_start()` **runtime-reattaches** a copy of `xPortSysTickHandle` (via
   `_xt_isr_attach(ETS_MAX_INUM, ...)`) that brackets the accumulate+reset with a
   generation counter `g_mono_ccount_seq`. (Pure linker override doesn't work: `port.o` is
   pulled wholesale → duplicate symbol; `--wrap` misses the intra-object reference; symbol
   isn't weak.) **This fixed the timing** — `bkwd=0`, `ahead` small & stable, `spin~10µs`.
7. **Spin-cap deadlock.** The spin cap used `mono_ccount()` for the timeout too. When the
   NMI preempts the tick handler mid-update, `mono_ccount()` freezes (returns the frozen
   accumulator) → the clock-based cap can never fire → **infinite NMI spin → full hang →
   watchdog** (the 0.5–5 s crashes). Fixed by making the cap an **iteration count**
   (`PWM_SPIN_MAX_ITERS`), which always advances.

---

## 5. THE REMAINING ISSUE (where you pick up)

**A fundamental, irreducible race in `mono_ccount()` causes a rare residual flicker.**

`mono_ccount()` = `g_esp_os_cpu_clk (accumulator) + soc_get_ccount() (live CCOUNT)`. The
tick handler updates these two pieces non-atomically (`acc += ccount;` then `CCOUNT = 0;`).
Our edge ISR is an **NMI** and can fire *between* those two writes. In that ~few-cycle
window the accumulator already includes the cycles but CCOUNT isn't zeroed → we count them
twice → `mono_ccount()` is off by **one tick (~10 ms)**. Because the ISR reads
`mono_ccount()` ~1M times/sec, this ~5-cycle window is hit ~once every 10–40 s.

The standard fix (seqlock retry) **can't be used**: an NMI that preempted the tick handler
can't spin waiting for it to finish (the handler is frozen until the NMI returns →
deadlock). And two separate memory writes can't be made atomic. So the window is
irreducible with the split accumulator+CCOUNT clock.

Mitigations already applied (help, don't eliminate): tightened/flipped the seq window so
the glitch is forward-only (`bkwd` now ~0 on the glitch lines, `ahead` shows `-10006`);
re-anchor rejects the ~10 ms jump so it can't corrupt `s_target`. Net effect now: a
glitched read makes the spin exit ~8 µs early → **one short/skipped pulse** → visible as a
micro-flick, worst on the very-dim settings the user tests (4 % duty, 8 µs pulse).

There's ALSO a second, smaller residual: profiler `ahead=[-118, …]` with `reanc/scap=1`
(~1/3 s) — our NMI itself being delayed ~118 µs, likely Wi-Fi NMI work and/or our
TSF-zeroing colliding with the MAC's use of TSF0. A late "off" edge stretches the dim
pulse → brief bright flash.

### The recommended fix (agreed direction, NOT yet implemented)

**Switch the timebase from CCOUNT to the free-running TSF counter — exactly what the SDK
PWM does, which is why it never flickers.** The TSF (`WDEVTSF0_TIME_LO`) is a single,
free-running, never-reset 1 µs counter — no accumulator, no reset, no race. Plan:
- Spin on `REG_READ(WDEVTSF0_TIME_LO)` instead of `mono_ccount()`.
- Arm **absolute** TSF compares (read TSF, set compare = TSF + wake) and **stop zeroing
  the TSF** — this also removes the TSF/MAC collision (the -118 µs issue) and is gentler
  on the sniffer (the SDK re-arms relative-to-live-TSF; we'd go absolute).
- `s_target` and gaps become TSF ticks (1 µs) instead of CCOUNT cycles. Delete
  `mono_clock.{c,h}` and its call, remove `s_cyc_shift` and the shift math (TSF is 1 µs,
  no CPU-freq dependence).
- **Cost: precision drops from ~12 ns to 1 µs.** This is the SDK's resolution, proven
  flicker-free. For a dim 8 µs pulse that's ±1 µs jitter — invisible. The 200 ns dream is
  abandoned because *this hardware has no glitch-free fine clock* (CCOUNT is the only fine
  one and it's tick-reset).

The user was leaning toward accepting this. **Confirm with them, then implement the TSF
switch.** It's contained: schedule compiler, edge table, LUTs, dedup, double-buffer, raw
path, protocol all stay unchanged — only the clock source and `wdev_arm` change.

Alternative if they insist on keeping CCOUNT precision: minimize the seq window to a
single-store flag (~1 cycle, ~1/min glitch) + absolute TSF arming for the -118 µs — but
the race is never truly zero. Not recommended.

---

## 6. Build / flash / test

- **Build:** `./build.sh` (bash) → `build/bulb-firmware.bin` (~468 KB). PowerShell's
  `build.ps1` trips on CMake stderr; use bash. Toolchain + SDK are already on PATH.
- **Flash (bulb on COM16, 74800 baud, its download mode):**
  ```
  esptool --chip esp8266 --port COM16 --baud 74800 --before default-reset --after hard-reset \
    write-flash --flash-mode dout --flash-size 4MB --flash-freq 40m \
    0x0 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin \
    0xd000 build/ota_data_initial.bin 0x10000 build/bulb-firmware.bin
  ```
  `default-reset` auto-enters the bootloader (DTR/RTS are wired). NVS (bulb id) untouched.
  If COM16 is "busy", the user's serial monitor has it — ask them to close it.
- **Console:** 74880 baud. **Host tests:** `cd test && make` (needs a hosted C compiler;
  on this box use clang inside the VS2022 dev shell — see history — `cc` isn't hosted).
- The user drives flashing/scope; don't auto-flash without a go-ahead, and the port is
  often held by their monitor.

---

## 7. Cleanup checklist (once flicker is closed)

- Set `PWM_DEBUG_DUMP` and `PWM_PROFILE` back to `0` in `config.h`.
- Remove the profiler block + `pwm_profile_task` + `s_prof_*` from `pwm_output.c`.
- If TSF switch: delete `mono_clock.{c,h}`, drop from `main/CMakeLists.txt`, remove
  `s_cyc_shift`/shift math, retune `PWM_AHEAD_TICKS`/spin cap for 1 µs ticks.
- Update `README.md` PWM section (currently reflects the FRC1 era in places).
- Re-run host tests; do a soak on hardware; confirm sniffer RX still works
  (`send_update.py`) and DFU2 still holds through an update.

---

## 8. Key files

`main/pwm_schedule.{c,h}`, `main/pwm_output.{c,h}`, `main/mono_clock.{c,h}`,
`main/config.h` (all the `PWM_*` tunables), `main/protocol.{c,h}` + `main/sniffer.c` +
`main/controller.{c,h}` (0x06 path), `main/app_main.c` (init order),
`test/test_pwm_schedule.c`, `test/test_protocol.c`, `tools/send_update.py`, `spec.txt`.
SDK tick handler reference: `../ESP8266_RTOS_SDK/components/freertos/port/esp8266/port.c`
(`xPortSysTickHandle`). SDK PWM reference (TSF register dance):
`../ESP8266_RTOS_SDK/components/esp8266/driver/pwm.c`.

Nothing is committed — all changes are in the working tree.
