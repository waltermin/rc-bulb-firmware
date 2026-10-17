#!/usr/bin/env python3
"""Generate (and optionally flash) an NVS image carrying a bulb's u8 id.

The firmware reads its id from NVS namespace "bulb", key "bulb_id". This tool
writes a matching NVS partition image so one identical app binary can be flashed
to every bulb while each keeps its own id.

Generate an image (--device picks the partition table; default bulb):
    python3 provision_id.py --id 7 --out nvs_7.bin
    python3 provision_id.py --device panel --id 80

Then flash it at the nvs partition offset it prints (0x9000 on both devices):
    esptool.py --chip esp8266 -p COM5 write_flash 0x9000 nvs_7.bin

Requires IDF_PATH to point at the device's SDK (ESP8266_RTOS_SDK for the bulb,
ESP-IDF v6 for the panel) so the NVS format matches the firmware (for the bundled
nvs_partition_gen.py). NVS partition size must match the nvs partition in devices/<device>/partitions.csv.
"""
import argparse
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CHIP = {"bulb": "esp8266", "panel": "esp32c3"}


def nvs_partition(device):
    """(offset, size) of the nvs partition in devices/<device>/partitions.csv."""
    path = os.path.join(HERE, "..", "devices", device, "partitions.csv")
    for line in open(path):
        f = [x.strip() for x in line.split("#")[0].split(",")]
        if len(f) >= 5 and f[0] == "nvs":
            return int(f[3], 0), int(f[4], 0)
    sys.exit(f"no nvs partition in {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True, help="bulb id (0..255)")
    ap.add_argument("--out", default=None, help="output .bin (default nvs_<id>.bin)")
    ap.add_argument("--device", default="bulb", choices=sorted(CHIP),
                    help="device whose partition table to use (default bulb)")
    ap.add_argument("--size", type=lambda x: int(x, 0), default=None,
                    help="nvs partition size (default: from the device's partitions.csv)")
    args = ap.parse_args()
    offset, size = nvs_partition(args.device)
    if args.size is None:
        args.size = size

    if not 0 <= args.id <= 255:
        ap.error("id must be 0..255")
    out = args.out or f"nvs_{args.id}.bin"

    idf = os.environ.get("IDF_PATH")
    if not idf:
        sys.exit("IDF_PATH not set; point it at the device's SDK checkout")
    gen = os.path.join(idf, "components", "nvs_flash",
                       "nvs_partition_generator", "nvs_partition_gen.py")
    if not os.path.exists(gen):
        sys.exit(f"nvs_partition_gen.py not found at {gen}")

    csv = ("key,type,encoding,value\n"
           "bulb,namespace,,\n"
           f"bulb_id,data,u8,{args.id}\n")

    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="") as tf:
        tf.write(csv)
        csv_path = tf.name

    try:
        cmd = [sys.executable, gen, "generate", csv_path, out, hex(args.size)]
        print("running:", " ".join(cmd))
        subprocess.run(cmd, check=True)
    finally:
        os.unlink(csv_path)

    print(f"\nwrote {out} (bulb_id={args.id})")
    print(f"flash with:\n    esptool.py --chip {CHIP[args.device]} write_flash {offset:#x} {out}")


if __name__ == "__main__":
    main()
