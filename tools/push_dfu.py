#!/usr/bin/env python3
"""Push a firmware image to a bulb that is in DFU mode.

The bulb, once in DFU mode, joins the configured AP, takes a DHCP lease with
hostname rc_light_dfu_<id>, and listens on DFU_TCP_PORT (default 3333). Find its
IP from your AP/DHCP server's lease table (look for that hostname), then:

    python3 push_dfu.py 192.168.4.23 ../build/bulb-firmware.bin

Wire header (little-endian):  magic u32 | version u8 | image_len u32 | sha256[32]
"""
import argparse
import hashlib
import socket
import struct
import sys

DFU_MAGIC = 0x686C4352  # "RClh"
DFU_HDR_VERSION = 0x01
DEFAULT_PORT = 3333

STATUS = {
    0: "OK — image accepted, bulb rebooting into new firmware",
    1: "bad header (magic/version/length rejected)",
    2: "esp_ota_begin failed",
    3: "receive error/timeout",
    4: "SHA-256 mismatch",
    5: "esp_ota_end validation failed",
    6: "esp_ota_set_boot_partition failed",
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", help="bulb IP address (in DFU mode)")
    ap.add_argument("image", help="firmware .bin to push")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        data = f.read()
    if not data:
        sys.exit("image is empty")

    digest = hashlib.sha256(data).digest()
    header = struct.pack("<IBI", DFU_MAGIC, DFU_HDR_VERSION, len(data)) + digest
    assert len(header) == 41

    print(f"pushing {len(data)} bytes (sha256={digest.hex()[:16]}...) to {args.host}:{args.port}")
    with socket.create_connection((args.host, args.port), timeout=args.timeout) as s:
        s.settimeout(args.timeout)
        s.sendall(header)
        s.sendall(data)
        # Bulb replies with a single status byte before rebooting.
        try:
            resp = s.recv(1)
        except socket.timeout:
            resp = b""

    if not resp:
        print("no status byte received (bulb may have rebooted before replying)")
        return
    code = resp[0]
    print(f"status {code}: {STATUS.get(code, 'unknown')}")
    sys.exit(0 if code == 0 else 1)


if __name__ == "__main__":
    main()
