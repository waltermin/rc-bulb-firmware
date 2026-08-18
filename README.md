# bulb-firmware

A minimal, ground-up firmware for the KAUF RGBWW (ESP8266) bulb, built directly
on **ESP8266_RTOS_SDK**. It replaces the stock ESPHome stack with a small,
purpose-built control path:

- **Instant default color** — the LEDs are driven to a configurable RGBWW value
  as the first action in `app_main`, before any radio is powered.
- **Broadcast control** — the bulb listens in **promiscuous mode** for 802.11
  beacons carrying a vendor-specific IE (OUI `52 43 68`). Each beacon holds a
  `LightUpdatePacket` with up to 11 `BulbEntry` records; the bulb applies the
  entry whose `bulb_id` matches its own NVS-provisioned `u8` id.
- **Fallback** — if no entry addressed to this bulb arrives within
  `FALLBACK_TIMEOUT_MS` (default 15 s), it reverts to the startup default.
- **DFU / OTA** — a beacon with `control_flags == 1` and `control_data == <my id>`
  drops promiscuous mode, joins a hard-coded AP, and receives a new firmware
  image over TCP into the inactive OTA slot (see *DFU* below).

## Hardware

| Channel | GPIO |
|---------|------|
| Red     | 4    |
| Green   | 12   |
| Blue    | 14   |
| Cold White | 5 |
| Warm White | 13 |

PWM runs at 250 Hz, capped at 80 % duty, gamma 2.8, with the stock per-channel
phase offsets — all configurable in `main/config.h`.

## Layout

```
main/
  config.h        all tunables (#defines)
  app_main.c      startup ordering + wiring
  pwm_output.*    5-channel PWM (gamma + max_power + phase)
  id_store.*      bulb id from NVS
  sniffer.*       promiscuous RX -> parser -> controller
  protocol.*      pure, host-testable frame/packet parser
  controller.*    applies colors, fallback timer, DFU trigger
  dfu.*           STA connect + DHCP hostname + TCP OTA server + rollback guard
partitions.csv    4 MB two-OTA table
test/             host unit tests for protocol.c
tools/            send_update.py, push_dfu.py, provision_id.py
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

After the first serial flash, all further updates go through **DFU** — no serial
needed.

## DFU / OTA

Reliability model (ESP8266_RTOS_SDK has **no** native app-rollback API):

1. On a DFU-addressed beacon the bulb leaves promiscuous mode, resets to the
   default color, sets its DHCP hostname to `rc_light_dfu_<id>`, and joins the AP
   defined by `DFU_AP_SSID` / `DFU_AP_PASS`.
2. It listens on `DFU_TCP_PORT` (3333). The pusher sends a 41-byte header
   (`magic u32 | version u8 | image_len u32 | sha256[32]`) then the raw image.
3. The image streams straight into the **inactive** OTA slot via `esp_ota_write`.
   The active slot is never touched, so a power loss at any point still boots the
   current firmware.
4. Before switching, the bulb checks the transport **SHA-256** *and* lets
   `esp_ota_end` validate the on-flash image. Only then does it flip the boot
   pointer (a single atomic `otadata` update) and reboot.
5. Optional NVS **boot-counter rollback guard** (`DFU_ROLLBACK_GUARD`): a fresh
   image must run healthy for `DFU_BOOT_VALIDATE_MS`, or the bootloader is pointed
   back at the previous slot after `DFU_BOOT_FAIL_THRESHOLD` failed boots.

Pushing a build once a bulb is in DFU mode (find its IP by the
`rc_light_dfu_<id>` DHCP lease on your AP):

```sh
python3 tools/push_dfu.py 192.168.4.23 build/bulb-firmware.bin
```

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

# ask bulb 7 to enter DFU mode
sudo python3 tools/send_update.py -i wlan0mon --dfu 7 --count 5
```

## Configuration

Everything an operator changes lives in `main/config.h`: default color, fallback
timeout, Wi-Fi channel, PWM parameters, DFU AP credentials/port, and the
rollback-guard settings. Wire-format constants (OUI, protocol version, max
entries) live in `main/protocol.h`.
