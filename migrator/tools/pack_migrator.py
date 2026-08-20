#!/usr/bin/env python3
"""Package a personalized migrator .bin.gz for one bulb id.

Assembles: [migrator combined image @0x0] + 0xFF pad + [payload header @0x110000]
+ [RTOS full-flash image @0x111000], then gzips it. Upload the result to the
bulb's ESPHome DFU/OTA web panel; eboot inflates and copies it to 0x0, boots the
migrator at 0x1000, which validates the payload (CRC-32) and rewrites low flash
into the RTOS layout. See ../README.md.

Build the migrator image ONCE first:
    cd bulb-firmware/migrator && pio run -e migrator
Then per bulb:
    python tools/pack_migrator.py --id 7
No recompile per id — only the NVS blob + header change.
"""
import argparse
import gzip
import os
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

# --- migrator payload contract (mirror src/payload_hdr.h) ---
MIG_MAGIC = 0x47494D4B      # 'KMIG'
MIG_VERSION = 1
HDR_ADDR = 0x110000
PAYLOAD_ADDR = 0x111000
SECTOR = 0x1000

# --- ESP8266 4m-no-FS layout facts (eagle.flash.4m.ld) ---
FS_START_OFF = 0x3FB000     # _FS_start - 0x40200000; OTA staging grows down from here
FREE_SKETCH_SPACE = 3690496 # web panel cap on the COMPRESSED upload (== ESP.getFreeSketchSpace())
NVS_OFFSET = 0x9000

FORBIDDEN_NAME_TOKENS = ("minimal", "wled", "-1m", "plf10", "plf12", "srf10", "rgbsw", "plug")


def sector_up(n: int) -> int:
    return (n + SECTOR - 1) & ~(SECTOR - 1)


def gen_nvs(idf_path: Path, provision_py: Path, bulb_id: int, out: Path) -> None:
    env = dict(os.environ, IDF_PATH=str(idf_path))
    cmd = [sys.executable, str(provision_py), "--id", str(bulb_id), "--out", str(out)]
    print("  nvs:", " ".join(cmd))
    subprocess.run(cmd, check=True, env=env)


def assemble_rtos_image(build_dir: Path, nvs_bin: bytes) -> bytes:
    """Contiguous 0x0.. image from flasher_args.json's flash_files, + personalized nvs."""
    import json
    fa = json.loads((build_dir / "flasher_args.json").read_text())
    # {offset: bytes} from the canonical flash map, plus the personalized nvs at 0x9000.
    placed = {int(off, 0): (build_dir / rel).read_bytes()
              for off, rel in fa["flash_files"].items()}
    placed[NVS_OFFSET] = nvs_bin

    length = sector_up(max(off + len(data) for off, data in placed.items()))
    img = bytearray(b"\xFF" * length)
    for off, data in placed.items():
        img[off:off + len(data)] = data
    return bytes(img)


def build_header(length: int, sector_count: int, payload_crc: int) -> bytes:
    body = struct.pack("<6I", MIG_MAGIC, MIG_VERSION, 0, length, sector_count, payload_crc)
    assert len(body) == 24
    hdr_crc = zlib.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("<I", hdr_crc)


def main() -> None:
    here = Path(__file__).resolve().parent           # bulb-firmware/migrator/tools
    migrator_root = here.parent                       # bulb-firmware/migrator
    fw_root = migrator_root.parent                     # bulb-firmware
    repo_root = fw_root.parent                          # recurse_hue

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True, help="bulb id (0..255)")
    ap.add_argument("--migrator-bin", type=Path,
                    default=migrator_root / ".pio" / "build" / "migrator" / "firmware.bin")
    ap.add_argument("--build-dir", type=Path, default=fw_root / "build",
                    help="RTOS firmware build/ (flasher_args.json + bins)")
    ap.add_argument("--idf-path", type=Path,
                    default=Path(os.environ.get("IDF_PATH", repo_root / "ESP8266_RTOS_SDK")))
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    if not 0 <= args.id <= 255:
        ap.error("id must be 0..255")
    out = args.out or (here.parent / f"kauf-bulb-4m-migrator-id{args.id}.bin")

    provision_py = fw_root / "tools" / "provision_id.py"
    for p in (args.migrator_bin, args.build_dir / "flasher_args.json", provision_py):
        if not Path(p).exists():
            sys.exit(f"missing: {p}")

    print(f"[pack] id={args.id}")
    # 1) personalized NVS
    with tempfile.TemporaryDirectory() as td:
        nvs_out = Path(td) / f"nvs_{args.id}.bin"
        gen_nvs(args.idf_path, provision_py, args.id, nvs_out)
        nvs_bin = nvs_out.read_bytes()
    if len(nvs_bin) != 0x4000:
        sys.exit(f"nvs blob is {len(nvs_bin)} bytes, expected 0x4000")

    # 2) RTOS full-flash image
    img = assemble_rtos_image(args.build_dir, nvs_bin)
    length = len(img)
    sector_count = length // SECTOR
    payload_crc = zlib.crc32(img) & 0xFFFFFFFF
    print(f"[pack] rtos image len=0x{length:X} ({length}) sectors={sector_count} crc=0x{payload_crc:08X}")

    # 3) header
    hdr = build_header(length, sector_count, payload_crc)

    # 4) migrator combined image
    M = Path(args.migrator_bin).read_bytes()
    if not (M[0] == 0xE9 and M[0x1000] == 0xE9):
        sys.exit("migrator-bin is not a combined eboot+app image (E9 at 0x0 and 0x1000)")
    if len(M) > HDR_ADDR:
        sys.exit(f"migrator image ({len(M)}) overruns payload header at 0x{HDR_ADDR:X}")

    # 5) uncompressed upload
    up = bytearray(b"\xFF" * (PAYLOAD_ADDR + length))
    up[0:len(M)] = M
    up[HDR_ADDR:HDR_ADDR + len(hdr)] = hdr
    up[PAYLOAD_ADDR:PAYLOAD_ADDR + length] = img

    # 6) RAW upload (NOT gzip). eboot's uzlib mis-inflates our large image (huge
    #    0xFF gap + payload) -- verified on-device: the migrator inflated fine but
    #    the payload came out as a pattern repeating every 0x6000 from ~0x1112DA.
    #    A raw 0xE9 image takes eboot's non-gzip copy path (SPIRead+SPIWrite, no
    #    uzlib), which is a dumb byte copy. It fits well under the size cap.
    blob = bytes(up)

    # 7) guards
    copy_end = PAYLOAD_ADDR + length            # eboot copies raw blob to 0..copy_end
    staged_src = FS_START_OFF - sector_up(len(blob))
    errs = []
    if len(blob) >= FREE_SKETCH_SPACE:
        errs.append(f"blob {len(blob)} >= free sketch space {FREE_SKETCH_SPACE}")
    if staged_src <= copy_end:
        errs.append(f"eboot self-clobber: staged_src 0x{staged_src:X} <= copy_end 0x{copy_end:X}")
    name_l = out.name.lower()
    if "bulb" not in name_l or "-4m" not in name_l:
        errs.append(f"filename '{out.name}' must contain 'bulb' and '-4m'")
    for tok in FORBIDDEN_NAME_TOKENS:
        if tok in name_l:
            errs.append(f"filename contains forbidden token '{tok}'")
    if errs:
        sys.exit("[pack] GUARD FAILED:\n  " + "\n  ".join(errs))

    out.write_bytes(blob)
    print(f"[pack] wrote {out}  (RAW, not gzipped)")
    print(f"       blob={len(blob)} (0x{len(blob):X})  cap={FREE_SKETCH_SPACE}")
    print(f"       staged_src=0x{staged_src:X}  copy_end=0x{copy_end:X}  "
          f"margin=0x{staged_src - copy_end:X}")


if __name__ == "__main__":
    main()
