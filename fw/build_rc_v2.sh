#!/bin/bash
# build_rc_v2.sh — build the rc_v2 OTA target as an rt_ota .rbl.
#
# rc_v2 is functionally identical to rc_bare EXCEPT it defaults the relay ON
# (RCSW_DEFAULT_RELAY=1) and carries version "rc_v2" — a visible, unambiguous
# post-swap signature for the DFU2 end-to-end test. Output: output/rc_v2.rbl,
# ready to serve to a switch running rc_bare.
#
# IMPORTANT: RCSW_VERSION is a -D compiler flag with no header dependency, so
# make will NOT rebuild the app objects when only USER_SW_VER changes. We delete
# the app objects first to force a clean recompile at the right version — the
# stale-object trap that once baked "rc_v2" into an "rc_bare" build.
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SDK="$REPO/vendor/OpenBK7231T_App/sdk/beken_freertos_sdk"
TC="$SDK/../OpenBK7231N/platforms/BK7231N/toolchain/windows/gcc-arm-none-eabi-4_9-2015q1/bin"
export PATH="/c/Users/rose/AppData/Local/Microsoft/WinGet/Links:$TC:$PATH"
export ARM_GCC_TOOLCHAIN="$TC/"
CFG="$REPO/fw/main/config.h"

cleanup() {
    # restore config.h + force the next (rc_bare) build to recompile cleanly
    sed -i 's/^#define RCSW_DEFAULT_RELAY    1/#define RCSW_DEFAULT_RELAY    0/' "$CFG"
    rm -f "$REPO"/vendor/fw/main/*.app.o "$SDK/libapp.a"
}
trap cleanup EXIT

# relay ON for rc_v2 — and VERIFY the sed matched: a silent no-op here would
# ship an "rc_v2" that does not default the relay ON.
sed -i 's/^#define RCSW_DEFAULT_RELAY    0/#define RCSW_DEFAULT_RELAY    1/' "$CFG"
grep -q '^#define RCSW_DEFAULT_RELAY    1' "$CFG" || { echo "ERROR: RCSW_DEFAULT_RELAY edit did not take in $CFG"; exit 1; }
# clean app objects so dfu2.c/user_main.c recompile at USER_SW_VER=rc_v2
rm -f "$REPO"/vendor/fw/main/*.app.o "$SDK/libapp.a"

cd "$SDK"
APP="$SDK/out/bsp.bin"
rm -f "$APP"
# The SDK make target's final full-image packaging step (beken_packager) is not
# on PATH and fails — irrelevant here, we only need the app bsp.bin it produces
# during compilation just before. Tolerate that failure; the freshly-(re)created
# bsp.bin below is the real check that compilation (at relay=1) succeeded.
make ROOT_DIR="$(pwd)" bk7238 -j8 USER_SW_VER=rc_v2 OBK_DIR="../../../../fw" || true

[ -f "$APP" ] || { echo "compile failed: $APP not produced"; exit 1; }

# Package as rt_ota RBL with the EXACT format the v1.0.15 bootloader expects,
# copied verbatim from the SDK's own build.sh: gzip + AES256 with the fixed
# key/iv baked into the bootloader (algo 0x0102 in the original rc_v2.rbl).
mkdir -p "$REPO/output"
"$SDK/tools/rtt_ota/rt_ota_packaging_tool_cli.exe" \
    -f "$(cygpath -w "$APP")" -o "$(cygpath -w "$REPO/output/rc_v2.rbl")" \
    -p app -c gzip -s aes -k 0123456789ABCDEF0123456789ABCDEF -i 0123456789ABCDEF -v rc_v2

echo "==> $REPO/output/rc_v2.rbl ($(stat -c%s "$REPO/output/rc_v2.rbl") bytes)"
