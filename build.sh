#!/usr/bin/env bash
# Build the bulb firmware with ESP8266_RTOS_SDK via CMake + Ninja.
#
# This drives CMake directly (not idf.py) and applies two workarounds needed on
# this Windows host:
#   1. .stubbin/ on PATH provides a stub `mconf-idf.exe` so the SDK's kconfig
#      init check passes (menuconfig itself is unavailable; a plain build uses
#      confgen.py). See .stubbin/README.md.
#   2. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 lets CMake 4.x configure the SDK's old
#      submodules (mbedtls etc.) that declare cmake_minimum_required < 3.5.
#
# Prereqs on PATH: xtensa-lx106-elf-gcc, cmake, ninja, python (with the SDK's
# requirements.txt installed: pyserial click future setuptools ...).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export IDF_PATH="${IDF_PATH:-$(cd "$here/../ESP8266_RTOS_SDK" && pwd)}"
export PATH="$here/.stubbin:$PATH"

# Stamp a unique build id into esp_app_desc.version via version.txt (the SDK reads
# it before falling back to git describe). This is the DFU2 build identity: the
# bulb and the update server both derive the 8-byte build id as sha256(version)[:8]
# (the toolchain does not populate app_elf_sha256). A fresh random token per build
# guarantees every image is distinguishable. Set BULB_BUILD_ID to pin it.
build_id="${BULB_BUILD_ID:-rc-$(python -c 'import secrets;print(secrets.token_hex(6))')}"
printf '%s\n' "$build_id" > "$here/version.txt"
echo "build id (version.txt): $build_id"

build_dir="$here/build"
mkdir -p "$build_dir"
cd "$build_dir"

cmake -G Ninja -DIDF_TARGET=esp8266 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$here"
ninja

echo
echo "artifacts in $build_dir:"
ls -1 bulb-firmware.bin bootloader/bootloader.bin partition_table/partition-table.bin
python - "$build_dir/bulb-firmware.bin" <<'PY'
import os, sys
s = os.path.getsize(sys.argv[1]); slot = 0xF0000
print(f"app: {s} bytes ({s/1024:.1f} KB), {100*s/slot:.1f}% of {slot//1024} KB OTA slot")
PY
