#!/usr/bin/env python3
"""Batch-build personalized migrator blobs for a set of bulb ids.

Thin wrapper around pack_migrator.py: for each requested id it invokes the
single-id packer (which generates the id's NVS, assembles the RTOS full-flash
image, prepends the CRC-32 header, and enforces every safety guard) and writes
the result into one output directory under the canonical per-id filename. The
heavy migrator image is built ONCE and reused; only the NVS + header change per
id, so this is just the single-id tool run in a loop.

Build the migrator image first (once, no recompile per id):
    cd bulb-firmware/migrator && pio run -e migrator

Then build a whole set:
    python tools/pack_migrator_range.py 1-55
    python tools/pack_migrator_range.py 1-10,20,30-35 --out-dir dist
    python tools/pack_migrator_range.py --start 1 --end 55

Output files are kauf-bulb-4m-migrator-id<N>.bin (the 'bulb'/'-4m' tokens are
required by pack_migrator's KaufHA filename guard), one per id, in --out-dir
(default: migrator/blobs, which .gitignore already excludes).

Exit status is non-zero if any id failed to build.
"""
import argparse
import subprocess
import sys
from pathlib import Path

# Canonical per-id filename. Contains 'bulb' and '-4m' (pack_migrator requires
# both) and none of its forbidden tokens; matches pack_migrator's own default.
FILENAME = "kauf-bulb-4m-migrator-id{id}.bin"


def parse_ids(spec: str) -> list:
    """Parse an id set like '1-55' or '1,2,5-10,42' into a list of ints."""
    ids = []
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if "-" in tok:
            lo_s, hi_s = tok.split("-", 1)
            lo, hi = int(lo_s), int(hi_s)
            if lo > hi:
                raise ValueError(f"range '{tok}' is backwards (start > end)")
            ids.extend(range(lo, hi + 1))
        else:
            ids.append(int(tok))
    return ids


def main() -> None:
    here = Path(__file__).resolve().parent           # migrator/tools
    migrator_root = here.parent                        # migrator
    pack = here / "pack_migrator.py"

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", nargs="?",
                    help="id set, e.g. '1-55' or '1,2,5-10,42' (or use --start/--end)")
    ap.add_argument("--start", type=int, help="range start (inclusive)")
    ap.add_argument("--end", type=int, help="range end (inclusive)")
    ap.add_argument("--out-dir", type=Path, default=migrator_root / "blobs",
                    help="output folder (default: migrator/blobs)")
    ap.add_argument("--keep-going", action="store_true",
                    help="continue after a failed id instead of stopping")
    ap.add_argument("--skip-existing", action="store_true",
                    help="skip ids whose output .bin already exists")
    # Pass-throughs to pack_migrator.py; only forwarded when explicitly given so
    # pack_migrator applies its own defaults otherwise.
    ap.add_argument("--migrator-bin", type=Path, help="override prebuilt migrator firmware.bin")
    ap.add_argument("--build-dir", type=Path, help="override RTOS build/ dir")
    ap.add_argument("--idf-path", type=Path, help="override IDF_PATH for NVS generation")
    args = ap.parse_args()

    # --- resolve the id set ---
    if args.ids and (args.start is not None or args.end is not None):
        ap.error("give either a positional id set OR --start/--end, not both")
    if args.ids:
        try:
            ids = parse_ids(args.ids)
        except ValueError as e:
            ap.error(str(e))
    elif args.start is not None and args.end is not None:
        if args.start > args.end:
            ap.error("--start must be <= --end")
        ids = list(range(args.start, args.end + 1))
    else:
        ap.error("specify ids: a positional set like '1-55', or --start N --end M")

    ids = sorted(set(ids))
    out_of_range = [i for i in ids if not (0 <= i <= 255)]
    if out_of_range:
        ap.error(f"ids out of range 0..255: {out_of_range}")
    if not ids:
        ap.error("no ids to build")

    if not pack.exists():
        sys.exit(f"missing packer: {pack}")

    args.out_dir.mkdir(parents=True, exist_ok=True)

    passthrough = []
    if args.migrator_bin:
        passthrough += ["--migrator-bin", str(args.migrator_bin)]
    if args.build_dir:
        passthrough += ["--build-dir", str(args.build_dir)]
    if args.idf_path:
        passthrough += ["--idf-path", str(args.idf_path)]

    print(f"[range] building {len(ids)} blob(s): ids {ids[0]}..{ids[-1]} -> {args.out_dir}")

    ok, failed, skipped = [], [], []
    for k, bid in enumerate(ids, 1):
        out = args.out_dir / FILENAME.format(id=bid)
        tag = f"[{k}/{len(ids)}] id={bid}"

        if args.skip_existing and out.exists():
            print(f"{tag} SKIP (exists: {out.name})")
            skipped.append(bid)
            continue

        cmd = [sys.executable, str(pack), "--id", str(bid), "--out", str(out), *passthrough]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode == 0 and out.exists():
            print(f"{tag} OK   -> {out.name} ({out.stat().st_size} bytes)")
            ok.append(bid)
        else:
            print(f"{tag} FAIL (pack_migrator exit {proc.returncode}) -- output below:")
            sys.stdout.write(proc.stdout)
            sys.stderr.write(proc.stderr)
            failed.append(bid)
            if not args.keep_going:
                sys.exit(f"[range] stopped at id {bid} "
                         f"({len(ok)} built). Use --keep-going to build the rest.")

    print(f"\n[range] done: {len(ok)} ok, {len(failed)} failed, {len(skipped)} skipped "
          f"-> {args.out_dir}")
    if failed:
        print(f"[range] failed ids: {failed}")
        sys.exit(1)


if __name__ == "__main__":
    main()
