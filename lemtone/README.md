# lemtone — Lemtone value → PWM duty + period

Pure C module (no SDK, no libc) that turns a Lemtone value into per-channel on-time
ticks and PWM periods. The firmware links it as a component for devices with
`DEVICE_LEMTONE 1` in `devices/<device>/device.cmake`, each with its own generated model
(`DEVICE_LEMTONE_MODEL`): the bulb `lemtone_model_default.c`, the panel
`lemtone_model_panel.c` (see *Panel* below). `main/controller.c` solves every
`0x07 LemtoneUpdate` entry addressed to the device.

| file | what |
|---|---|
| `lemtone.h/.c` | solver, period selection, quantisation, OKLab conversions |
| `lemtone_model_default.c`, `lemtone_model.json` | bulb calibration — **generated** by `calibration/export_lemtone_model.py` |
| `lemtone_model_panel.c`, `lemtone_model_panel.json`, `test/lemtone_vectors_panel.h` | panel calibration and its vectors — **generated** by `calibration/export_panel_model.py` |
| `CMakeLists.txt` | firmware component (only `lemtone.c` + the model; built at -O2) |
| `test/test_lemtone.c`, `test/lemtone_vectors.h` | host tests; vectors **generated** by `calibration/lemtone_ref.py` (float64 reference of the same algorithm) |
| `web/lemtone_tool.template.html`, `web/lemtone_wasm.c` | validation page source; `web/lemtone_tool.html` is **generated** (wasm, CRI code/tables and model spectra inlined) |
| `web/cri.js`, `web/cri_tables.json`, `test/cri_cases.json` | CIE 13.3 CRI for the page; tables and reference cases **generated** from colour-science by `web/gen_cri_tables.py`, checked by `web/check_cri.mjs` |
| `build.py` | `python build.py [test\|web] [model…]` — host tests, wasm build, wasm-vs-vectors check (node), for every model present (`web/lemtone_tool.html` bulb, `web/lemtone_tool_panel.html` panel) |

## The Lemtone value

Floats, device independent except fidelity:

- **a, b** — OKLab chromaticity: the colour's OKLab a and b at lightness L = 1 (a/L,
  b/L), so colour is independent of brightness. Real colours span about a −0.83…0.79,
  b −1.12…0.27. Anything the bulb cannot make (including imaginary colours) is
  desaturated toward white.
- **lm** — luminous flux in lumens; 0 = off. The calibration measured illuminance at the
  bench sensor, so the model carries `lm_per_y`, anchored on the rated output: **1000 lm
  at WW + CW both at the duty cap** (Kauf's rating; a placeholder until a real flux
  measurement — changing it rescales brightness only). This bulb then makes ≈ 600 lm
  with WW alone, ≈ 48 / 122 / 28 lm red / green / blue alone, and ≈ 0.003 lm at its
  dimmest step.
- **F** — fidelity 0…1: how much of an unsaturated colour comes from the whites (1)
  versus RGB (0). Never changes the colour or the lumens, only how the spectrum is built.

The wire encoding (11-bit a, b; 12-bit log lumens; 6-bit F) lives in
`main/protocol.h`, not here; the solver only sees the decoded floats.

**Duties** out of the solver are *light-equivalent*: each PWM pulse emits about
0.5–0.75 µs more light than its tick count (fitted from the n=10 low-duty ladder), so
`lemtone_quantize` uses `ticks = round(duty · 2ⁿ − pulse_us)`. Its `cmd_duty`
(`ticks / 2ⁿ`) is what to hand `pwm_output_set_raw` / `setraw` (scale 1); the PWM
compiler rounds it back to exactly `ticks`.

## Solve

Model: `XYZ(d) = Σ dᵢ · gᵢ(d) · Tᵢ(dᵢ)` — per-unit-duty colour `T` (thermal-session
ladder with a fixed warm correction), interaction `g` (spec2, 18 terms per channel).
The target (a, b, lm) becomes XYZ once; the solve runs in XYZ.

1. Closed form on a linear model `Σ dᵢ Pᵢ` (as in `lemtone.html`): whites split so their
   mix lands where the target projects onto the CW–WW xy line (a duty share, weighted by
   each white's X+Y+Z per unit duty), RGB from a 3×3 inverse makes up the rest, feasible
   white range solved directly, fidelity picks the point in it.
2. Frozen coefficients: `Pᵢ` re-evaluated at the duties just found and step 1 redone,
   first without `g`, then with it. Fidelity therefore holds on the full model (asking
   for WW-at-cap's own colour at F1 returns WW alone).
3. Infeasible: bisect saturation along the **OKLab line** toward the white the whites alone
   make at the target's place on their line and lumens, at the same lumens (`DESAT`). RGB
   corrects as far as it has headroom, so a colour desaturates only once the lumens outgrow
   what R, G, B can add, and more as they grow. (Not the straight OKLab line between CW and
   WW: midway it runs ~0.01 off what the whites make and is itself unreachable once RGB runs
   out.) If even that white can't reach the lumens, bisect the lumens there (`CLAMP`).

Exact solves: 3–5 model evaluations on the bulb (median 9 on the panel, which snaps its
whites afterwards). Desaturating ones: ~30; worst ~100 (bulb), ~190 (panel).

## Periods

Per channel: shortest period from `n_min` that gives `res_ticks` of on-time, capped by
total lumens (default > 180 lm → 2^10, > 60 lm → 2^12, else 2^14). Stateless — no
hysteresis — so the same value always gives the same periods. Thresholds are
`LEMTONE_PERIOD_DEFAULT` and are meant to be tuned by eye with the tool page.

## Regenerating

```
cd calibration
python export_lemtone_model.py --thermal ../captures/thermal_20260926_104045_bulb55_main \
    --mix ../captures/mix_20260926_090726_bulb55_main \
    --pulse ../captures/session_20260926_030918_bulb55 --out ../bulb-firmware/lemtone
    # --rated-lm / --rated-duty change the lumen anchor (default 1000 lm at 0,0,0,0.8,0.8)
python lemtone_ref.py ../bulb-firmware/lemtone/lemtone_model.json \
    --vectors ../bulb-firmware/lemtone/test/lemtone_vectors.h
cd ../bulb-firmware/lemtone && python build.py
# only if the CIE tables are missing (needs pip install colour-science):
python web/gen_cri_tables.py
```

## CRI on the page

`lemtone_model.json` also carries each channel's warm-corrected spectrum at the knots
(380–780 nm, 5 nm; tools only — the C model stays XYZ). The page rebuilds the solve's
spectrum as `Σ light_dutyᵢ · gᵢ · spectrumᵢ(light_dutyᵢ)` and computes CIE 13.3 Ra and
R1–R14. `cri.js` matches colour-science to 0.01 Ra on standard illuminants. Predicted vs
measured on 40 white mixes from the mix session (a different run from the one the spectra
come from): Ra within ±1.1 (rms 0.5), R9 rms 3 (worst 7).

## Tool page

Open `web/lemtone_tool.html` in Chrome/Edge (Web Serial). Pick a chromaticity on the
OKLab a/b plane at L = 1 (or hue/chroma, or the CCT slider), set lumens and F. The value
is encoded to the 0x07 wire entry and decoded back exactly as the bulb does, and the solve
(duties, periods, ticks, predicted output, Δu′v′, CRI, spectrum) runs on the decoded
value. Connect to the base station, turn on *Live send*, and pick **Send as**:

- `lt <id> a b lm F` — the base station broadcasts the Lemtone value (0x07) and the bulb
  solves it;
- `setraw <id> … 1 <periods>` — the page's own solve as raw duties (0x06).

The two should give identical light; flipping between them is the end-to-end check of
the on-bulb solver. The *Bulb ids* boxes take one id or a range (first to last); a range is sent
as `ltrange`, while `setraw` (one target on the base station) addresses only the first id. Changing
the range while live clears the old one first. *Stop* sends `ltclr` for the range / `setraw off`. The periods panel can pin
every channel to one period (setraw only) for A/B comparisons.

## Panel

The panel (ESP32-C3, `devices/panel`) has R, G, B strings plus a tunable-white driver IC fed
DIM (= ww + cw) and CCT (= ww / (ww + cw)) on hardware PWM at a fixed 3906.25 Hz carrier and
12-bit duty (`main/pwm_mix.c`, `main/port/esp32c3/pwm_hw_ledc.c`). Same
solver and model form, no special case for the whites in the solve itself; what differs is
hardware description in the model (all 0 / off for the bulb):

- `duty_max` 1 (100 % is safe), `white_sum_max` 1: ww + cw ≤ 1, one string's worth.
- `white_grid_log2` 12, `white_dim_min` 48: the white pair is snapped onto its DIM/CCT grid
  (steps of 1/4096, the driver PWM's resolution; the firmware warns at boot if they differ) and
  R, G, B are re-solved for the light the unsnapped solve made. DIM below 48/4096 (= 3/256) is
  never used: the captures, made at 1/256 steps, found the driver erratic at 1–2 of those.
  `ticks[CW]`/`ticks[WW]` are then DIM/CCT steps; `cmd_duty` is the logical ww/cw pair the
  mixer turns back into exactly those steps. The whites were *measured* at 1/256 steps only:
  between those points the model interpolates the measured DIM curve, so the 12-bit grid is
  an extrapolation. If the driver turns out to decode only 8 bits, export with
  `WHITE_GRID_LOG2 = 8` (`calibration/export_panel_model.py`); no recapture needed.
- `white_by_dim`: the whites' per-unit colour is read at DIM, not each white's own duty (the
  driver sets one current for the pair). Knots are the 24 measured DIM levels.
- `pulse_by_period`, `pulse_n`: each RGB pulse *loses* light, more at long periods
  (≈ −1.6 µs at n=10 to −6 µs at n=14 for red, −9 µs for green); per channel and period.
- `lm_per_y`: 4000 lm (the panel's rating) at CW alone, full DIM.
- Static thermal state: the knots are scaled to half the warm-vs-cold probe gain of
  `thermal1` (R −0.7 %, G −0.5 %, B +1.4 %, whites −0.7 %), the best of none / half / full on
  `valid1`. The panel drifts only ~2–3 % cold → hot (the bulb's red: 27 %).

Measured end to end on panel 120 (`captures/panel120/valid3`, 71 Lemtone targets solved by
`lemtone_ref.py` with this model, sent as setraw): **ΔY rms 1.3 % (max 2.5 %), Δu′v′ rms 0.0014
(max 0.0040; 0.0007 above 500 lm)**. Targets below ~5 lm are outside the spectroradiometer's
range at the bench distance and are not verified. `valid3` was solved before the white split
and the desaturation anchor became the generic ones above: same model, but some targets now get
other duties, which have not been re-measured. See `calibration/README.md`
(*Panel*) for the captures and fit.

