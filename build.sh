#!/usr/bin/env bash
# Build one device's firmware:  ./build.sh bulb   |   ./build.sh panel
#
# Devices live in devices/<name>/ (device.cmake says which SDK target). Each builds
# into its own build-<device>/ with its own sdkconfig.<device>.
#
#   esp8266 target (bulb)  ESP8266_RTOS_SDK via CMake + Ninja, driven directly (not
#     idf.py), with two workarounds needed on Windows hosts:
#       1. .stubbin/ on PATH provides a stub `mconf-idf.exe` so the SDK's kconfig
#          init check passes (menuconfig itself is unavailable; a plain build uses
#          confgen.py). See .stubbin/README.md.
#       2. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 lets CMake 4.x configure the SDK's old
#          submodules (mbedtls etc.) that declare cmake_minimum_required < 3.5.
#     SDK: $ESP8266_SDK_PATH, else ../ESP8266_RTOS_SDK. Prereqs on PATH:
#     xtensa-lx106-elf-gcc, cmake, ninja, python (with the SDK's requirements.txt).
#
#   anything else (panel)  ESP-IDF v6: export it first (. $IDF_PATH/export.sh), then
#     this runs idf.py. (IDF v6 refuses to run under MSys/Git Bash; use build.ps1
#     on Windows.)
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
device="${1:-}"
if [ -z "$device" ] || [ ! -f "$here/devices/$device/device.cmake" ]; then
    echo "usage: $0 <$(ls "$here/devices" | tr '\n' '|' | sed 's/|$//')>" >&2
    exit 2
fi
setting() { sed -n "s/^set($1[[:space:]]\{1,\}\([^[:space:])]*\).*/\1/p" "$here/devices/$device/device.cmake"; }
target="$(setting DEVICE_TARGET)"
project="$(setting DEVICE_PROJECT)"
echo "device $device ($target) -> $project.bin"

# Stamp a unique build id into esp_app_desc.version via version.txt (the SDK reads
# it before falling back to git describe). This is the DFU2 build identity: the
# device and the update server both derive the 8-byte build id as
# sha256(version)[:8] (the ESP8266 toolchain does not populate app_elf_sha256). A
# fresh random token per build guarantees every image is distinguishable. Set
# BULB_BUILD_ID to pin it.
build_id="${BULB_BUILD_ID:-rc-$(python -c 'import secrets;print(secrets.token_hex(6))')}"
printf '%s' "$build_id" > "$here/version.txt"
echo "build id (version.txt): $build_id"

build_dir="$here/build-$device"
mkdir -p "$build_dir"

if [ "$target" = "esp8266" ]; then
    sdk="${ESP8266_SDK_PATH:-$here/../ESP8266_RTOS_SDK}"
    [ -d "$sdk/components/esp8266" ] || { echo "ESP8266_RTOS_SDK not found (set ESP8266_SDK_PATH)" >&2; exit 1; }
    export IDF_PATH="$(cd "$sdk" && pwd)"
    export PATH="$here/.stubbin:$PATH"
    (cd "$build_dir" &&
        cmake -G Ninja -DDEVICE="$device" -DIDF_TARGET=esp8266 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$here" &&
        ninja)
else
    command -v idf.py >/dev/null || { echo "idf.py not found: export ESP-IDF v6 first" >&2; exit 1; }
    idf.py -C "$here" -B "$build_dir" -DDEVICE="$device" build
fi

echo
echo "artifacts in $build_dir:"
(cd "$build_dir" && ls -1 "$project.bin" bootloader/bootloader.bin partition_table/partition-table.bin)
python - "$build_dir/$project.bin" "$here/devices/$device/partitions.csv" <<'PY'
import os, sys
s = os.path.getsize(sys.argv[1])
slot = next(int(l.split(",")[4], 16) for l in open(sys.argv[2]) if l.strip().startswith("ota_0"))
print(f"app: {s} bytes ({s/1024:.1f} KB), {100*s/slot:.1f}% of {slot//1024} KB OTA slot")
PY
