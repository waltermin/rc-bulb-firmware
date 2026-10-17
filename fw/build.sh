#!/bin/bash
# build.sh — build the bare-SDK rc-switch firmware.
#
# Uses the beken_freertos_sdk + ARM toolchain + packager vendored under
# vendor/OpenBK7231T_App (gitignored; reproduce with its submodules). We build
# ONLY our app (fw/) against the SDK — no OpenBeken OS. The SDK's application.mk
# is patched to `-include $(OBK_DIR)/fw.mk` instead of OpenBeken.mk
# (see vendor/openbeken-rcswitch.patch).
#
# Run from repo root:  bash fw/build.sh
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SDK="$REPO/vendor/OpenBK7231T_App/sdk/beken_freertos_sdk"
TC="$SDK/../OpenBK7231N/platforms/BK7231N/toolchain/windows/gcc-arm-none-eabi-4_9-2015q1/bin"
export PATH="/c/Users/rose/AppData/Local/Microsoft/WinGet/Links:$TC:$PATH"
export ARM_GCC_TOOLCHAIN="$TC/"

cd "$SDK"
# RCSW_VERSION is a -D flag with no header dependency: make will NOT recompile
# app objects when only USER_SW_VER changes, silently baking a stale version
# (and build id) into the image. Force a clean app recompile — it's 6 files.
rm -f "$REPO"/vendor/fw/main/*.app.o "$SDK/libapp.a"

# The SDK make target's final step is its OWN full-image packaging via a
# 'beken_packager' helper that isn't on PATH — it fails, but only AFTER the app
# has compiled + linked. We do our own packaging below, so tolerate that failure
# and use the freshly-(re)built app image as the real proof the compile passed.
APP="$SDK/out/bk7231_bsp.bin"
rm -f "$APP"
make ROOT_DIR="$(pwd)" bk7238 -j8 USER_SW_VER=rc_bare OBK_DIR="../../../../fw" || true
[ -f "$APP" ] || { echo "compile failed: $APP not produced"; exit 1; }

# package the full physical image (v1.0.15 bootloader + app) for serial flashing
cd tools/beken_packager
MT="$USERPROFILE/AppData/Local/Temp/obk_pk"; mkdir -p "$MT"
export TEMP="$(cygpath -w "$MT")" TMP="$(cygpath -w "$MT")"
python ./beken_packager_wrapper -i 7 -s 0x200000 >/dev/null 2>&1 || true
./beken_packager.exe tmp.json
mkdir -p "$REPO/output"
cp all_2M.1220.bin "$REPO/output/rcsw_bare_full_2M.bin"
echo "==> $REPO/output/rcsw_bare_full_2M.bin  ($(stat -c%s all_2M.1220.bin) bytes)"
echo "flash with: python tools/flash_obk_full.py output/rcsw_bare_full_2M.bin"
