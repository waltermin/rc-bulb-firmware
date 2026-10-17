#!/bin/bash
# build_docker.sh — build the bare-SDK rc-switch firmware on macOS/Linux.
#
# Same steps as build.sh (which targets the Windows toolchain), but run inside
# OpenBeken's own build container (vendor/OpenBK7231T_App/docker: Ubuntu 20.04
# amd64 + i386 libs) plus Arm's GCC 10.3-2021.10, which is what Beken's
# prebuilt SDK libraries (librwnx_bk7238.a etc.) were compiled with; the
# bundled GCC 4.9 can't link their LTO objects. The Linux beken_packager runs
# there too. On Apple Silicon this needs Docker/OrbStack with amd64 emulation
# (Rosetta).
#
# Run from repo root:  bash fw/build_docker.sh [USER_SW_VER]   (default rc_bare)
#   CLEAN=1 bash fw/build_docker.sh   rebuilds the whole SDK first (needed once
#   after switching toolchains: cached SDK archives from another GCC won't LTO-link)
# Output: output/rcsw_bare_full_2M.bin (v1.0.15 bootloader + app, physical format)
#         output/<USER_SW_VER>.rbl     (rt_ota OTA image for DFU2, same app)
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
VER="${1:-rc_bare}"
BASE="obk-build:ubuntu20.04"
IMAGE="obk-build:gcc10.3"
GCC_URL="https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2"

if ! docker image inspect "$BASE" >/dev/null 2>&1; then
    docker build --platform linux/amd64 -t "$BASE" "$REPO/vendor/OpenBK7231T_App/docker"
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker build --platform linux/amd64 -t "$IMAGE" - <<DOCKERFILE
FROM $BASE
USER root
RUN apt-get update && apt-get install -y bzip2 && \\
    wget -q -O /tmp/gcc.tar.bz2 "$GCC_URL" && \\
    tar -xjf /tmp/gcc.tar.bz2 -C /opt && rm /tmp/gcc.tar.bz2
DOCKERFILE
fi

# Mount the repo at its real path: the SDK's cached .d dependency files hold
# absolute host paths, so a different mount point breaks make.
docker run --rm --platform linux/amd64 --user "$(id -u):$(id -g)" \
    -v "$REPO:$REPO" -w "$REPO" -e REPO="$REPO" -e VER="$VER" -e CLEAN="${CLEAN:-0}" "$IMAGE" bash -euc '
SDK=$REPO/vendor/OpenBK7231T_App/sdk/beken_freertos_sdk
TC=/opt/gcc-arm-none-eabi-10.3-2021.10/bin
export ARM_GCC_TOOLCHAIN=$TC/ PATH=$TC:$PATH

cd $SDK
[ "$CLEAN" = 1 ] && make clean >/dev/null
# RCSW_VERSION is a -D flag with no header dependency: force a clean app
# recompile so the version (and build id) baked into the image is current.
rm -f $REPO/vendor/fw/main/*.app.o $SDK/libapp.a

# On Linux the make target finishes by running the SDK packaging step
# (tools/beken_packager/beken_packager_wrapper), which writes the full physical
# image (v1.0.15 bootloader + app) to out/all_2M.1220.bin. On Windows that step
# fails, which is why build.sh packages with the .exe separately.
APP=$SDK/out/bk7231_bsp.bin
FULL=$SDK/out/all_2M.1220.bin
rm -f $APP $FULL
# ENCRYPT: the local application.mk patch points at the Windows encrypt.exe.
make ROOT_DIR="$(pwd)" bk7238 -j8 USER_SW_VER=$VER OBK_DIR="../../../../fw" \
    ENCRYPT=./tools/crc_binary/encrypt_n
[ -f $APP ] || { echo "compile failed: $APP not produced"; exit 1; }
[ -f $FULL ] || { echo "packaging failed: $FULL not produced"; exit 1; }

mkdir -p $REPO/output
cp $FULL $REPO/output/rcsw_bare_full_2M.bin
echo "==> output/rcsw_bare_full_2M.bin ($(stat -c%s $FULL) bytes, version $VER)"

# OTA image in the exact rt_ota RBL format the v1.0.15 bootloader expects
# (gzip + AES256 with the fixed key/iv baked into it), as build_rc_v2.sh does.
$SDK/tools/rtt_ota/rt_ota_packaging_tool_cli-x64 -f $APP -o $REPO/output/$VER.rbl \
    -p app -c gzip -s aes -k 0123456789ABCDEF0123456789ABCDEF -i 0123456789ABCDEF -v $VER >/dev/null
echo "==> output/$VER.rbl ($(stat -c%s $REPO/output/$VER.rbl) bytes)"
'
