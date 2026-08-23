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
//               └─ payload = { u8 packet_tag; ... }
//
// The first payload byte after the OUI is a packet tag that selects the format
// of the bytes that follow. The defined packet types are:
//
//   0x01 LightUpdate (DEPRECATED) — u8-per-channel colors + inline DFU trigger:
//     struct LightUpdatePacket {
//       u8 packet_tag;     // 0x01
//       u8 control_flags;  // 1 => control_data is a bulb id to put into DFU mode
//       u8 control_data;
//       u8 entry_count;    // 0..11
//       BulbEntry entries[entry_count];
//     }
//     Only processed when PROTO_ENABLE_LEGACY_LIGHT_UPDATE is set (default off);
//     superseded by 0x03 (color) + 0x04 (DFU). Otherwise rejected as unknown.
//
//   0x02 PreciseLightUpdate — one bulb, float32-per-channel colors:
//     struct PreciseLightUpdate {
//       u8  packet_tag;    // 0x02
//       u8  bulb_id;
//       f32 r, g, b, ww, cw;   // little-endian, nominally [0.0, 1.0]
//     }
//
//   0x03 LightUpdateV2 — like LightUpdate but with the DFU/control fields
//        removed (DFU is now its own BulbCommand):
//     struct LightUpdateV2 {
//       u8 packet_tag;     // 0x03
//       u8 entry_count;    // 0..11
//       BulbEntry entries[entry_count];
//     }
//
//   struct BulbEntry { u8 bulb_id, r, g, b, ww, cw; }  // 6 bytes (0x01 and 0x03)
//
//   0x04 BulbCommand — remote management (config writes, DFU) with anti-replay:
//     struct BulbCommandPacket {
//       u8  packet_tag;     // 0x04
//       u32 seq;            // process only if strictly > highest seq seen so far
//       u8  bulb_id_start;  // addressed to ids [start, start + bounds] inclusive
//       u8  bulb_id_bounds; // 0 => only bulb_id_start
//       u8  cmd;            // command tag
//       ...                 // command payload
//     }
//     cmd 0x00 EnterDfuMode: no payload.
//     cmd 0x01 SetConfig:    u16 key; u8 length; u8 value[length].
//     cmd 0x02 Reboot:       no payload.
//   The seq gate ("highest seq since power-on") is stateful and handled by the
//   caller (the sniffer), not this pure parser: parse exposes seq + is_command.
//
// Color-bearing packets (0x01/0x02/0x03) resolve to "no update for me" or a
// normalized [0,1] color; commands (0x04) resolve to a DFU or config request.
// protocol_parse_beacon() hides the wire encoding behind bulb_parse_result_t.

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

// Packet tags — the first payload byte after the OUI. See the wire-format
// comment at the top of this file.
#define PROTO_TAG_LIGHT_UPDATE 0x01u          // (deprecated) u8-per-channel table + DFU
#define PROTO_TAG_PRECISE_LIGHT_UPDATE 0x02u  // single bulb, f32-per-channel
#define PROTO_TAG_LIGHT_UPDATE_V2 0x03u       // u8-per-channel table, no DFU
#define PROTO_TAG_BULB_COMMAND 0x04u          // remote management (config / DFU)
#define PROTO_TAG_SIZE 1u                     // width of the packet tag itself

// Whether the deprecated 0x01 LightUpdate is still processed. New firmware
// ignores it (spec: superseded by 0x03 + 0x04); define to 1 to keep accepting
// it (e.g. from a base station that also drives older bulbs). Gate lives here so
// the pure parser stays self-contained; override with -D at build time.
#ifndef PROTO_ENABLE_LEGACY_LIGHT_UPDATE
#define PROTO_ENABLE_LEGACY_LIGHT_UPDATE 0
#endif

#define PROTO_MAX_ENTRIES 11u           // ESP8266 firmware / beacon-size limit
#define PROTO_ENTRY_SIZE 6u             // bulb_id + 5 color channels

// LightUpdate fixed header: packet_tag + control_flags + control_data + entry_count.
#define PROTO_LIGHT_UPDATE_HDR_SIZE 4u
// LightUpdateV2 fixed header: packet_tag + entry_count.
#define PROTO_LIGHT_UPDATE_V2_HDR_SIZE 2u
// PreciseLightUpdate total size: packet_tag + bulb_id + 5 * f32.
#define PROTO_PRECISE_CHANNELS 5u
#define PROTO_PRECISE_SIZE (PROTO_TAG_SIZE + 1u + PROTO_PRECISE_CHANNELS * 4u)  // 22

#define PROTO_CTRL_FLAG_DFU 0x01u       // control_flags value that requests DFU mode

// BulbCommand (0x04). Fixed header: packet_tag + seq(u32) + start(u8) +
// bounds(u8) + cmd(u8) = 8 bytes, followed by a per-command payload.
#define PROTO_BULB_CMD_HDR_SIZE 8u
#define PROTO_CMD_ENTER_DFU 0x00u        // cmd: enter DFU mode (no payload)
#define PROTO_CMD_SET_CONFIG 0x01u       // cmd: set a config key (u16 key, u8 len, value[len])
#define PROTO_CMD_REBOOT 0x02u           // cmd: reboot the device (no payload)
#define PROTO_SETCONFIG_HDR_SIZE 3u      // key(2) + length(1), before the value bytes
#define PROTO_CONFIG_VALUE_MAX 64u       // largest config value we accept (bounds RX buffers)

// ---- parse result -----------------------------------------------------------

typedef struct {
    bool valid;          // frame is a well-formed packet for our protocol
    bool dfu_requested;  // DFU addressed to my_id (0x01 control field, or 0x04 EnterDfuMode)
    bool has_entry;      // a color addressed to my_id was present
    // Color to apply, normalized to [0.0, 1.0], only meaningful when has_entry.
    // LightUpdate/V2 u8 channels are scaled by 1/255; PreciseLightUpdate's f32
    // channels are passed through (clamped to [0,1], NaN treated as 0).
    float r, g, b, ww, cw;

    // BulbCommand (0x04) fields. is_command is set for any well-formed command
    // regardless of addressing, and seq is then valid — the caller tracks the
    // highest seq seen (anti-replay) since that state cannot live in this pure
    // parser. dfu_requested / has_config are set only when the command is
    // addressed to my_id.
    bool     is_command;
    uint32_t seq;
    bool     reboot_requested;   // Reboot command addressed to my_id
    bool     has_config;         // SetConfig addressed to my_id
    uint16_t config_key;         // 16-bit config key tag
    uint8_t  config_len;         // length of config_value, <= PROTO_CONFIG_VALUE_MAX
    const uint8_t *config_value; // -> value bytes within `frame`; copy before reuse
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
