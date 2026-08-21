#!/usr/bin/env python3
"""Broadcast light-control beacons for testing the bulb firmware.

Two packet types are supported (selected by the packet tag, the first payload
byte after the OUI):
    0x01 LightUpdate        --entry id:r,g,b,ww,cw   (u8 channels, 0..255)
    0x02 PreciseLightUpdate --precise id:r,g,b,ww,cw (f32 channels, 0.0..1.0)

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

    # Drive bulb id 7 to full-red with a PreciseLightUpdate (float channels):
    sudo python3 send_update.py -i wlan0mon --precise 7:1.0,0,0,0,0

    # Request that bulb id 7 enter DFU mode:
    sudo python3 send_update.py -i wlan0mon --dfu 7 --count 5
"""
import argparse
import struct
import sys
import time

try:
    from scapy.all import (RadioTap, Dot11, Dot11Beacon, Dot11Elt, sendp)
except ImportError:
    sys.exit("scapy is required: pip install scapy")

VENDOR_OUI = bytes([0x52, 0x43, 0x68])
TAG_LIGHT_UPDATE = 0x01
TAG_PRECISE_LIGHT_UPDATE = 0x02
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


def parse_precise(spec):
    """'id:r,g,b,ww,cw' (floats) -> PreciseLightUpdate body bytes (tag..cw)."""
    bulb_id, colors = spec.split(":")
    vals = [float(x) for x in colors.split(",")]
    if len(vals) != 5:
        raise ValueError(f"precise '{spec}' needs 5 color values (r,g,b,ww,cw)")
    bulb_id = int(bulb_id)
    if not 0 <= bulb_id <= 255:
        raise ValueError(f"bulb id out of range in '{spec}'")
    return bytes([TAG_PRECISE_LIGHT_UPDATE, bulb_id]) + struct.pack("<5f", *vals)


def build_frame_from_body(body):
    ie_info = VENDOR_OUI + body
    # Dot11Beacon contributes the 12-byte fixed params (timestamp/interval/cap);
    # our single vendor IE (ID=221) is the ONLY tagged element, as the spec requires.
    frame = (RadioTap() /
             Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff", addr2=BSSID, addr3=BSSID) /
             Dot11Beacon(cap="ESS") /
             Dot11Elt(ID=221, info=ie_info))
    return frame


def build_light_update(entries, control_flags, control_data):
    body = bytes([TAG_LIGHT_UPDATE, control_flags, control_data, len(entries)])
    for e in entries:
        body += e
    return build_frame_from_body(body)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-i", "--iface", required=True, help="monitor-mode interface")
    ap.add_argument("--entry", action="append", default=[],
                    help="LightUpdate (0x01) bulb entry 'id:r,g,b,ww,cw', u8 (repeatable)")
    ap.add_argument("--precise", default=None,
                    help="PreciseLightUpdate (0x02) 'id:r,g,b,ww,cw', floats 0.0..1.0")
    ap.add_argument("--dfu", type=int, default=None,
                    help="request DFU mode for this bulb id (LightUpdate control field)")
    ap.add_argument("--interval", type=float, default=0.5, help="seconds between frames")
    ap.add_argument("--count", type=int, default=0, help="frames to send (0 = forever)")
    args = ap.parse_args()

    # A PreciseLightUpdate is a distinct packet type and carries no entry table
    # or control/DFU fields, so it is mutually exclusive with --entry/--dfu.
    if args.precise is not None and (args.entry or args.dfu is not None):
        ap.error("--precise cannot be combined with --entry or --dfu")

    if args.precise is not None:
        frame = build_frame_from_body(parse_precise(args.precise))
        desc = f"precise={args.precise}"
    else:
        entries = [parse_entry(s) for s in args.entry]
        if len(entries) > MAX_ENTRIES:
            ap.error(f"at most {MAX_ENTRIES} entries per packet")

        control_flags, control_data = 0, 0
        if args.dfu is not None:
            control_flags, control_data = CTRL_FLAG_DFU, args.dfu

        if not entries and args.dfu is None:
            ap.error("provide at least one --entry, --precise, or --dfu")

        frame = build_light_update(entries, control_flags, control_data)
        desc = f"entries={len(entries)}, dfu={args.dfu}"

    print(f"sending {len(frame)}-byte beacon on {args.iface} ({desc})")

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
