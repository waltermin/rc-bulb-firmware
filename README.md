# rc-switch-firmware

A ground-up firmware for the **Tuya T1-2S-NL** light switch (BK7238) that
joins the RC light ecosystem: it listens in monitor mode for the same
beacon-broadcast protocol as `rc-bulb-firmware` (802.11 beacons carrying a
single vendor IE, OUI `52 43 68`) and acts on two relay-specific packets:

- **`0x40` RelaysUpdate** — a 32-byte bitfield over the shared device id
  space; a relay with id N sets its relay to bit N. Idempotent desired-state
  broadcast, rebroadcast continuously by the base station.
- **`0x41` RelaysLedUpdate** — same, driving the status LED.
- **`0x04` BulbCommand** — SetConfig / Reboot honored (shared id space);
  EnterDfuMode ignored (relays only have pull-based DFU2).
- **`0x05` Dfu2Request** — honored when `device_type` is `0x02` (relay).

Behavior: both outputs go to their configured **DEFAULT** states at power-on
(before the radio starts) and revert to DEFAULT whenever no packet of the
corresponding type is heard for 15 s. The two fallback timers are independent.
**The physical button has no function.**

See `spec.txt` in rc-bulb-firmware for the protocol document of record
(0x40/0x41 + the `device_type` field are added by `docs/spec-relay-additions.patch`).

## Hardware

| Signal | BK7238 pin |
|--------|------------|
| Relay  | P9 (active-high) |
| Status LED | P6 |
| Button | P8 (unused by design; held as pulled-up input) |
| Console / flash | UART1 (TX1/RX1), 115200 |

Stock flash layout is preserved (stock Beken bootloader kept at 0x0, app at
0x11000, OTA download partition at 0x10D000) so serial recovery always works.

## Layout

```
main/
  config.h        all tunables (pins, channel, defaults, timeouts, config keys)
  protocol.{c,h}  pure, host-testable frame/packet parser (no SDK includes)
  controller.{c,h} pure dual-default/dual-fallback-timer state machine
src/
  main.cpp        platform glue: monitor-mode RX -> parser -> controller -> pins
test/
  test_protocol.c host unit tests for parser + controller
  run_tests.py    builds with cc/gcc/clang or `pip install ziglang`
tools/
  flash_via_jig.py  flash through the BKJIG ESP32-S3 UART bridge
```

The parser and controller are pure C with no SDK dependencies; the platform
layer is built on LibreTiny (PlatformIO `platform = libretiny`, board
`t1-2s`), which supplies the ARM toolchain, the vendored Beken SDK (whose
`bk_wlan_*` monitor-mode API we call directly), and bootloader-compatible
image packaging.

## Build & test

```sh
python test/run_tests.py            # host unit tests
pio run -e t1-2s                    # firmware -> .pio/build/t1-2s/firmware.uf2
python tools/flash_via_jig.py       # flash via the bench jig
```

## Bench testing with BKJIG

The ESP32-S3 jig doubles as a base station (`RELAYS`, `RELAYSLED`, `BCAST`,
`AUTO` commands transmit protocol beacons on the configured channel), and its
`MON` command reads this firmware's UART1 console. Console keys: `s` = stats
(frame counters + current outputs), `d` = hexdump next protocol frame,
`0`-`9` = temporarily switch device id for addressing tests.
