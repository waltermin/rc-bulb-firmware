#!/usr/bin/env python3
"""Broadcast LightUpdatePacket beacons for testing the bulb firmware.

Requires a Wi-Fi interface in MONITOR mode on the same channel the bulb sniffs
(WIFI_CHANNEL in config.h, default 1) and root privileges. Uses scapy.

Set the channel first, e.g. on Linux:
    sudo ip link set wlan0 down
    sudo iw dev wlan0 set type monitor
    sudo ip link set wlan0 up
    sudo iw dev wlan0 set channel 1

Examples:
    # Drive bulb id 7 to red, repeating every 500 ms:
    sudo python3 send_update.py -i wlan0mon --entry 7:255,0,0,0,0

    # Drive several bulbs in one packet:
    sudo python3 send_update.py -i wlan0mon --entry 1:255,0,0,0,0 --entry 2:0,255,0,0,0

    # Request that bulb id 7 enter DFU mode:
    sudo python3 send_update.py -i wlan0mon --dfu 7 --count 5
"""
import argparse
import sys
import time

try:
    from scapy.all import (RadioTap, Dot11, Dot11Beacon, Dot11Elt, sendp)
except ImportError:
    sys.exit("scapy is required: pip install scapy")

VENDOR_OUI = bytes([0x52, 0x43, 0x68])
PROTO_VERSION = 0x01
CTRL_FLAG_DFU = 0x01
MAX_ENTRIES = 11
BSSID = "52:43:68:00:00:01"  # arbitrary; base-station address


def parse_entry(spec):
    """'id:r,g,b,ww,cw' -> bytes(6)."""
    bulb_id, colors = spec.split(":")
    vals = [int(x) for x in colors.split(",")]
    if len(vals) != 5:
        raise ValueError(f"entry '{spec}' needs 5 color values (r,g,b,ww,cw)")
    out = [int(bulb_id)] + vals
    for v in out:
        if not 0 <= v <= 255:
            raise ValueError(f"value out of range in '{spec}'")
    return bytes(out)


def build_packet(entries, control_flags, control_data):
    body = bytes([PROTO_VERSION, control_flags, control_data, len(entries)])
    for e in entries:
        body += e
    ie_info = VENDOR_OUI + body
    # Dot11Beacon contributes the 12-byte fixed params (timestamp/interval/cap);
    # our single vendor IE (ID=221) is the ONLY tagged element, as the spec requires.
    frame = (RadioTap() /
             Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff", addr2=BSSID, addr3=BSSID) /
             Dot11Beacon(cap="ESS") /
             Dot11Elt(ID=221, info=ie_info))
    return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-i", "--iface", required=True, help="monitor-mode interface")
    ap.add_argument("--entry", action="append", default=[],
                    help="bulb entry 'id:r,g,b,ww,cw' (repeatable)")
    ap.add_argument("--dfu", type=int, default=None,
                    help="request DFU mode for this bulb id")
    ap.add_argument("--interval", type=float, default=0.5, help="seconds between frames")
    ap.add_argument("--count", type=int, default=0, help="frames to send (0 = forever)")
    args = ap.parse_args()

    entries = [parse_entry(s) for s in args.entry]
    if len(entries) > MAX_ENTRIES:
        ap.error(f"at most {MAX_ENTRIES} entries per packet")

    control_flags, control_data = 0, 0
    if args.dfu is not None:
        control_flags, control_data = CTRL_FLAG_DFU, args.dfu

    if not entries and args.dfu is None:
        ap.error("provide at least one --entry or --dfu")

    frame = build_packet(entries, control_flags, control_data)
    print(f"sending {len(frame)}-byte beacon on {args.iface} "
          f"(entries={len(entries)}, dfu={args.dfu})")

    sent = 0
    try:
        while args.count == 0 or sent < args.count:
            sendp(frame, iface=args.iface, verbose=False)
            sent += 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    print(f"sent {sent} frames")


if __name__ == "__main__":
    main()
