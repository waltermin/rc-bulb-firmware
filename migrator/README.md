# Trampoline migrator (ESPHome → RTOS-SDK, over the DFU panel)

One-time, over-the-air migrator that upgrades a stock-ESPHome Kauf RGBWW **BR30**
bulb (4 MB / esp07s) to the `bulb-firmware` ESP8266_RTOS_SDK firmware, delivered
through the bulb's ESPHome **DFU/OTA web upload panel** — no serial access needed.
**Verified working end-to-end on the bench** (stock ESPHome → DFU upload → the
bulb migrates itself and reboots into the new firmware automatically).

## How it works

The stock bulb boots **eboot** (Arduino bootloader) at `0x0` + an ESPHome sketch
at `0x1000`. The DFU panel accepts any image whose first byte is `0xE9`, and on
reboot eboot copies it to `0x0`, then boots the app at `0x1000`. We exploit that:

The uploaded **raw** `.bin` lands (via eboot's copy) as this flash image:

```
0x000000  eboot (from THIS PlatformIO build)          }
0x001000  migrator sketch  <- eboot boots this         } combined migrator image
0x00????  0xFF pad
0x110000  KMIG payload header (magic,len,CRC-32)       } pre-positioned so the payload
0x111000  personalized RTOS full-flash image (0x83000) } source never overlaps 0x0..0x83000
```

The migrator (booted at `0x1000`):
1. validates the payload **CRC-32** — reading flash via the SDK's `spi_flash_read`
   (the ROM `SPIRead` mis-reads addresses >1 MB while the cache is on);
2. from an **IRAM-resident, cache-off** routine, copies the RTOS image
   (`0x111000` → `0x0`), writing the 3 bootloader sectors **last** with
   verify-readback;
3. resets via the **hardware watchdog** (`ets_wdt_enable()` + spin). The RTC
   "SWRST" is only a CPU-level reset and hangs from the migrator's post-write
   state; a `REASON_WDT_RST` fully re-inits the cache/SPI controller so the freshly
   written RTOS bootloader boots cleanly.

Sources: `src/main.cpp` (validate), `src/flash_rewrite.cpp` (IRAM engine),
`src/leds.h` (LED map).

### Why raw, not gzip

The DFU panel also accepts `.bin.gz` (eboot inflates it), **but eboot's `uzlib`
inflater corrupts our large image** — verified on-device: the migrator part
inflated fine, then the payload came out as a pattern repeating every `0x6000`.
A raw `0xE9` upload takes eboot's non-gzip path (a plain `SPIRead`+`SPIWrite`
copy, no `uzlib`). At 1.65 MB it's well under the DFU size cap, so `pack_migrator.py`
emits raw `.bin`.

## Build (once) + package (per bulb)

The migrator binary is built **once**; only the per-id NVS + header change, so
there's no recompile per bulb.

```sh
cd bulb-firmware/migrator
pio run -e migrator                    # -> .pio/build/migrator/firmware.bin (eboot+app)
python tools/pack_migrator.py --id 7   # -> kauf-bulb-4m-migrator-id7.bin  (raw, 1.65 MB)
```

`pack_migrator.py` generates the personalized NVS (`../tools/provision_id.py`,
needs `IDF_PATH`), assembles the RTOS image from `../build/flasher_args.json`
(bootloader/parttable/otadata/app + the id's NVS at `0x9000`), prepends the CRC-32
header, concatenates onto the prebuilt migrator image padded to `0x110000`, and
enforces guards (fits the DFU size cap; no eboot self-clobber; filename passes the
KaufHA product/`-4m` checks). The RTOS firmware must be built **DOUT** (matches the
stock flash mode; `../sdkconfig.defaults`).

## Migrate a bulb

Upload the raw `.bin` at `http://<bulb-ip>/` → DFU/OTA panel (or
`curl -F 'file=@kauf-bulb-4m-migrator-id7.bin' http://<bulb-ip>/update`).

Diagnostic LEDs (all capped at **192/255** duty):
- **solid blue** — migrating (held through the whole write via the Timer1 NMI);
- **solid green** — success (shown for the few-second watchdog pause);
- **flashing red** — failure (bad payload at validation, or a write/verify error).

The bulb then reboots itself into the RTOS firmware — **no power cycle needed**.
On the console at **74880 baud** you'll see: eboot copy → migrator banner →
`payload CRC OK -> migrating` → ~5 s write → a short WDT pause → the RTOS-SDK
bootloader log → the app printing its bulb id. (The garbage right at the WDT
reset is just the baud transition.)

On a validation failure the migrator does **nothing destructive** (flashing red;
recover via serial).

## ⚠️ Risk profile (minimal safety net, per project decision)

Once eboot boots the migrator, stock ESPHome is already overwritten — there is
**no OTA fallback**. The migrator's own flash code (`0x1000`+) overlaps the RTOS
app destination (`0x10000`+), so the moment the app write begins the migrator
corrupts its own on-flash image. **A power loss anytime during the ~5 s flash
write (not just the final bootloader sectors) leaves the bulb needing serial
recovery** — observed once on the bench during an accidental power cut. Bench
power is stable so this is a non-issue for trials; for sealed units this residual
brick risk is accepted.

Hardening option (not built): a bare-metal micro-migrator small enough to live
entirely in flash sectors 1–2 (skipped until the final write) would survive a
mid-write reset and retry idempotently, shrinking the unrecoverable window to the
3-sector bootloader write. Consider before any large fleet rollout.

## Recovery (bench, COM15)

Re-enter download mode (GPIO0→GND + reset; COM15 has no auto-reset) and reflash
stock, or the RTOS image directly:
```sh
# back to stock ESPHome:
esptool.py --chip esp8266 -p COM15 --before no_reset --after hard_reset \
  write_flash 0x0 ../../kauf-rgbww-bulbs/bench-artifacts/bench-bulb-stock-4m.factory.bin
# or straight to RTOS (per ../build/flasher_args.json offsets) + nvs at 0x9000
```
Tip: take a full backup first — `esptool.py ... read_flash 0 0x400000 backup.bin`.

## Notes for a fleet rollout (re-confirm before sealed units)

1. **Back up each unit's flash** (`read_flash 0 0x400000`) before its DFU — there
   is no serial recovery in the field.
2. The eboot **self-clobber guard** in `pack_migrator.py` is `4m`-no-filesystem
   specific (it assumes `_FS_start` = `0x3FB000`). A field build with SPIFFS/
   LittleFS moves FS down and can violate it — re-confirm the field layout.
3. First RTOS boot selects `ota_0` and reports the provisioned id.
4. The `192/255` LED cap protects the channels during the diagnostic — keep it.
