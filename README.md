# bulb-firmware

A minimal, ground-up firmware for the KAUF RGBWW (ESP8266) bulb, built directly
on **ESP8266_RTOS_SDK**. It replaces the stock ESPHome stack with a small,
purpose-built control path:

- **Instant default color** — the LEDs are driven to a configurable RGBWW value
  as the first action in `app_main`, before any radio is powered.
- **Broadcast control** — the bulb listens in **promiscuous mode** for 802.11
  beacons carrying a vendor-specific IE (OUI `52 43 68`). The first payload byte
  after the OUI is a **packet tag** selecting the format that follows:
  `0x01` **LightUpdate** holds up to 11 `BulbEntry` records with `u8` channels;
  `0x02` **PreciseLightUpdate** targets one bulb with `f32` channels. Either way
  the bulb applies the color whose `bulb_id` matches its own NVS-provisioned id.
- **Fallback** — if no entry addressed to this bulb arrives within
  `FALLBACK_TIMEOUT_MS` (default 15 s), it reverts to the startup default.
- **DFU2 / OTA (pull-based)** — a `0x05 Dfu2Request` beacon advertises an update
  for a bulb-id range (with AP creds, an update-server IP/port, and a target build
  id). A bulb in range whose own build id differs joins the AP, connects **out** to
  the server, pulls the image into its inactive OTA slot, and reboots (see *DFU2*
  below).

## Hardware

| Channel | GPIO |
|---------|------|
| Red     | 4    |
| Green   | 12   |
| Blue    | 14   |
| Cold White | 5 |
| Warm White | 13 |

PWM is driven by a custom software engine (`pwm_output.*` + `pwm_schedule.*`) on
the ESP8266 FRC1 hardware timer, so it needs no radio and the LEDs light at boot.
Each channel gets its own duty, phase, and PWM rate: periods are powers of two in
200 ns ticks, so all channels are harmonics that stack into one repeatable edge
table walked by an IRAM-resident ISR. Fine (200 ns) ticks keep the dimmest pulses
sharp instead of quantizing them to 1 µs. Capped at 80 % duty, gamma 2.8; the
default rate is ~1.2 kHz (`2^12` ticks) — all configurable in `main/config.h`.

## Layout

```
main/
  config.h        all tunables (#defines)
  app_main.c      startup ordering + wiring
  pwm_output.*    5-channel PWM engine (curve + max_power; FRC1 ISR + GPIO)
  pwm_schedule.*  pure per-channel duty/phase/period -> edge-table compiler
  id_store.*      bulb id from NVS
  sniffer.*       promiscuous RX -> parser -> controller (+ build-id gate for DFU2)
  protocol.*      pure, host-testable frame/packet parser
  controller.*    applies colors, fallback timer, DFU2 trigger
  dfu2.*          STA connect + TCP OTA CLIENT (pull) + rollback guard
partitions.csv    4 MB two-OTA table
test/             host unit tests for protocol.c
tools/            send_update.py, provision_id.py, dfu2_server.py,
                  dfu1_* (legacy push-DFU tools for old bulbs)
```

## Build & flash

Requires ESP8266_RTOS_SDK, the xtensa-lx106 toolchain, CMake, Ninja, and Python
(with the SDK's `requirements.txt` installed, incl. `setuptools` for
`pkg_resources`).

The simplest path is the checked-in script, which drives CMake+Ninja directly
and applies the workarounds documented below:

```sh
./build.sh          # -> build/bulb-firmware.bin (+ bootloader, partition table)
```

Or manually:

```sh
export IDF_PATH=/path/to/ESP8266_RTOS_SDK
export PATH="$PWD/.stubbin:$PATH"          # stub mconf-idf (see note 1)
mkdir build && cd build
cmake -G Ninja -DIDF_TARGET=esp8266 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..   # note 2
ninja
```

A verified clean build produces a **~457 KB** app — about 48 % of the 960 KB OTA
slot.

Flash the three images over serial the first time (offsets from `partitions.csv`):

```sh
esptool.py --chip esp8266 -p <PORT> write_flash \
  0x0     build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x10000 build/bulb-firmware.bin
```

**Build-environment notes (Windows / CMake 4.x):**
1. The SDK's kconfig init insists on an `mconf-idf` executable on `PATH` (or a
   native host gcc). `menuconfig` is not used by a normal build — a plain build
   generates config via `confgen.py` — so `.stubbin/mconf-idf.exe` (a trivial
   stub) satisfies the check. See `.stubbin/README.md`.
2. `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` lets CMake 4.x configure the SDK's older
   submodules (mbedtls etc.) whose `cmake_minimum_required` predates 3.5.

If you have a full ESP-IDF-style tools install, `idf.py set-target esp8266 &&
idf.py build && idf.py -p <PORT> flash monitor` also works.

Provision each bulb's id (one identical app image works for all bulbs):

```sh
python3 tools/provision_id.py --id 7 --out nvs_7.bin
esptool.py --chip esp8266 -p <PORT> write_flash 0x9000 nvs_7.bin
```

After the first serial flash, all further updates go through **DFU2** — no serial
needed.

## DFU2 / OTA (pull-based)

Each build carries a unique **build id**: `build.sh` writes a random token to
`version.txt`, the SDK stamps it into the app descriptor (`esp_app_desc.version`),
and both the bulb and the update server derive the 8-byte wire id as
`sha256(version)[:8]`. (The ESP8266 toolchain does not populate `app_elf_sha256`,
so we hash the version string instead.)

Rollout flow (ESP8266_RTOS_SDK has **no** native app-rollback API):

1. Start the update server with the image; it prints the build id and a
   ready-to-paste base-station command:
   ```sh
   python3 tools/dfu2_server.py build/bulb-firmware.bin --range 1-100 \
       --ssid "RC-Update" --pass "swordfish"
   #  -> dfu2 1 100 RC-Update swordfish 192.168.4.2 3333 <build_id_hex>
   ```
2. Paste that on the base station. It broadcasts a `0x05 Dfu2Request` beacon
   continuously (id range + AP creds + server IP/port + target build id) until
   `dfu2 off` / `stop`.
3. A bulb in range whose own build id differs leaves promiscuous mode, joins the
   advertised AP (DHCP hostname `rc_light_dfu_<id>`), connects **out** to the
   server, and sends a hello (`magic | fmt | bulb_id | build_id[8]`).
4. The server replies with a 49-byte header
   (`magic u32 | version u8 | image_len u32 | sha256[32] | build_id[8]`) then the
   raw image. The bulb streams it straight into the **inactive** OTA slot via
   `esp_ota_write`; the active slot is never touched, so a power loss at any point
   still boots the current firmware.
5. Before switching, the bulb verifies the advertised build id, the transport
   **SHA-256**, *and* lets `esp_ota_end` validate the on-flash image. Only then
   does it flip the boot pointer (a single atomic `otadata` update) and reboot.
6. Once running the new build, its build id matches the still-broadcasting
   request, so it ignores it — the rollout is idempotent and self-terminating. A
   bulb that fails at any step reboots into its untouched current firmware and
   retries on the next broadcast it hears.
7. Optional NVS **boot-counter rollback guard** (`DFU_ROLLBACK_GUARD`): a fresh
   image must run healthy for `DFU_BOOT_VALIDATE_MS`, or the bootloader is pointed
   back at the previous slot after `DFU_BOOT_FAIL_THRESHOLD` failed boots.

The server tracks each bulb's connection, progress, and result plus the overall
rollout (mirrored to `dfu2_state.json`).

**Packet size:** the ESP8266 Wi-Fi stack only delivers promiscuous frames under
128 bytes, so the `Dfu2Request` body is capped at 70 bytes (`PROTO_DFU2_MAX_BODY`).
The SSID + password + build id must fit; the base station **rejects** an over-long
request rather than broadcast an undeliverable frame. Use a short update-network
SSID/passphrase.

**Legacy bulbs:** bulbs still running the old push-DFU firmware are updated with
the retained `tools/dfu1_*` scripts + the base-station `dfu` / `dfurange` commands
(that is also how a legacy bulb is first brought onto this DFU2 firmware).

XIP note: SDK flash routines run from IRAM with the cache + interrupts disabled
per op, so `esp_ota_write` is XIP-safe. The PWM timer ISR is also IRAM-resident,
so the LEDs keep running through the update.

## Tests

The protocol parser is pure C with no SDK dependencies:

```sh
cd test && make        # builds with cc/gcc/clang and runs the vectors
```

## Testing the control path on the bench

Put a Wi-Fi NIC into monitor mode on channel 1 (`WIFI_CHANNEL`) and:

```sh
# drive bulb 7 red; stop after a while to watch it fall back to default
sudo python3 tools/send_update.py -i wlan0mon --entry 7:255,0,0,0,0

# advertise a DFU2 update for bulbs 1..25 (build id from dfu2_server.py's banner)
sudo python3 tools/send_update.py -i wlan0mon \
    --dfu2 1-25 RC-Update swordfish 192.168.4.2 3333 <build_id_hex>
```

## Configuration

Everything an operator changes lives in `main/config.h`: default color, fallback
timeout, Wi-Fi channel, PWM parameters, DFU2 timeouts, and the rollback-guard
settings (the DFU AP credentials now arrive in the `Dfu2Request` packet, not from
config). Wire-format constants (OUI, protocol/packet tags, max entries,
`PROTO_DFU2_*`) live in `main/protocol.h`.
