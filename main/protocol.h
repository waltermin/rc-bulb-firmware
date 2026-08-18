// protocol.h — pure parser for the broadcast light-control protocol.
//
// This header and protocol.c intentionally depend on NOTHING from the
// ESP8266_RTOS_SDK so the parser can be compiled and unit-tested on a host
// (see test/test_protocol.c). All wire-format constants live here as the
// single source of truth; firmware config.h may reference them.
//
// Wire format (little-endian, packed):
//
//   802.11 beacon management frame
//     ├─ MAC header            24 bytes
//     ├─ beacon fixed params   12 bytes (timestamp 8, interval 2, capability 2)
//     └─ tagged IEs
//          └─ vendor-specific IE (0xDD) — MUST be the only IE
//               ├─ tag        1 byte  (0xDD)
//               ├─ length     1 byte
//               ├─ OUI        3 bytes (0x52 0x43 0x68)
//               └─ payload = LightUpdatePacket
//
//   struct LightUpdatePacket {
//     u8 version;        // 0x01
//     u8 control_flags;  // 1 => control_data is a bulb id to put into DFU mode
//     u8 control_data;
//     u8 entry_count;    // 0..11
//     BulbEntry entries[entry_count];
//   }
//
//   struct BulbEntry { u8 bulb_id, r, g, b, ww, cw; }  // 6 bytes

#ifndef BULB_PROTOCOL_H
#define BULB_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- wire constants ---------------------------------------------------------

#define IEEE80211_HDR_LEN 24u
#define BEACON_FIXED_LEN 12u            // timestamp(8) + interval(2) + capability(2)
#define IEEE80211_IE_HDR_LEN 2u         // tag(1) + length(1)
#define IEEE80211_FCS_LEN 4u            // trailing frame-check sequence (may or may not be present)

#define IE_TAG_VENDOR 0xDDu

#define VENDOR_OUI_0 0x52u
#define VENDOR_OUI_1 0x43u
#define VENDOR_OUI_2 0x68u
#define VENDOR_OUI_LEN 3u

#define PROTO_VERSION 0x01u
#define PROTO_MAX_ENTRIES 11u           // ESP8266 firmware / beacon-size limit
#define PROTO_ENTRY_SIZE 6u             // bulb_id + 5 color channels
#define PROTO_HEADER_SIZE 4u            // version + control_flags + control_data + entry_count

#define PROTO_CTRL_FLAG_DFU 0x01u       // control_flags value that requests DFU mode

// ---- parse result -----------------------------------------------------------

typedef struct {
    bool valid;          // frame is a well-formed packet for our protocol
    bool dfu_requested;  // control_flags==DFU && control_data==my_id
    bool has_entry;      // an entry with bulb_id==my_id was present
    uint8_t r, g, b, ww, cw;  // only meaningful when has_entry is true
} bulb_parse_result_t;

// Parse a raw 802.11 frame (as delivered by the promiscuous RX path).
//
//   frame     : pointer to the first byte of the 802.11 frame (MAC header)
//   frame_len : number of valid bytes at `frame` (may include a 4-byte FCS)
//   my_id     : this bulb's id
//
// Returns a result with valid=false if the frame is not one of ours. All reads
// are bounds-checked against frame_len; a malformed/truncated frame is rejected
// rather than over-read.
bulb_parse_result_t protocol_parse_beacon(const uint8_t *frame, size_t frame_len, uint8_t my_id);

#ifdef __cplusplus
}
#endif

#endif  // BULB_PROTOCOL_H
