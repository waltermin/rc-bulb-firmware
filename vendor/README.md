# Vendored dependencies

## libretiny/

A working copy of [LibreTiny](https://github.com/libretiny-eu/libretiny)
**v1.13.0** with one local fix required to build the beken-72xx core for the
BK7238. `platformio.ini` points `platform = symlink://vendor/libretiny` so the
tree is used in place and the patch takes effect immediately.

`vendor/libretiny/` is intentionally **not** committed to this repo (it is a
large third-party tree; see `.gitignore`). Reproduce it with:

```sh
git clone --depth 1 --branch v1.13.0 \
    https://github.com/libretiny-eu/libretiny vendor/libretiny
git -C vendor/libretiny apply ../libretiny-psk-stub.patch
```

### Why the patch

The registry build of LibreTiny 1.13.0 fails to compile
`cores/beken-72xx/arduino/libraries/WiFi/WiFiSTA.cpp`: `WiFiClass::psk()`
dereferences `struct wpa_config`, which the BK7238 BDK keeps opaque
(forward-declared only), so the field access is an "invalid use of incomplete
type" error. The patch stubs the stored-passphrase getter to return `""`.
Nothing in this firmware reads a stored PSK (DFU2 credentials arrive in the
`0x05` packet), so the stub is behavior-preserving for our use.

`libretiny-psk-stub.patch` is the exact diff, applied from inside
`vendor/libretiny/`.
