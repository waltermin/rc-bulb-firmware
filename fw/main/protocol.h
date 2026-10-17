// protocol.h — pure parser for the broadcast relay-control protocol.
//
// This header and protocol.c intentionally depend on NOTHING from the vendor
// SDK so the parser can be compiled and unit-tested on a host (see
// test/test_protocol.c). All wire-format constants live here as the single
// source of truth; firmware config.h may reference them.
//
// This is the relay-device sibling of rc-bulb-firmware's protocol.{h,c} and
// keeps its frame validation byte-for-byte compatible:
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
// Packet tags this device acts on:
//
//   0x40 RelaysUpdate — desired relay state for the whole shared id space:
//     struct RelaysUpdatePacket {
//       u8 packet_tag;          // 0x40
//       u8 relay_states[32];    // bit N = desired relay state of device id N
//     }                         // bit N at relay_states[N/8], bit N%8 (LSB-first)
//
//   0x41 RelaysLedUpdate — same, but drives the status LED:
//     struct RelaysLedUpdatePacket {
//       u8 packet_tag;          // 0x41
//       u8 led_states[32];
//     }
//
//   0x04 BulbCommand — remote management, shared with bulbs. SetConfig and
//        Reboot are honored; EnterDfuMode (push-DFU) is ignored — relay
//        devices only ever had pull-based DFU2.
//
//   0x05 Dfu2Request — pull-based OTA. The first payload byte is the target
//        device_type; relay devices act only on PROTO_DEVICE_TYPE_RELAY (0x02)
//        and treat other types as valid-but-not-ours. (The field was formerly
//        fmt_version == 0x01, which now denotes "KAUF bulb".)
//
// Color packets (0x01/0x02/0x03/0x06) are recognized as ecosystem frames (valid =
// true, so diagnostics can count them) but carry no effect for a relay.

#ifndef RELAY_PROTOCOL_H
#define RELAY_PROTOCOL_H

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

// Packet tags — the first payload byte after the OUI.
#define PROTO_TAG_LIGHT_UPDATE 0x01u          // bulbs only (deprecated there too)
#define PROTO_TAG_PRECISE_LIGHT_UPDATE 0x02u  // bulbs only
#define PROTO_TAG_LIGHT_UPDATE_V2 0x03u       // bulbs only
#define PROTO_TAG_BULB_COMMAND 0x04u          // shared remote management
#define PROTO_TAG_DFU2_REQUEST 0x05u          // pull-based OTA advertisement
#define PROTO_TAG_RAW_LIGHT_UPDATE 0x06u      // bulbs only (raw duties + PWM periods)
#define PROTO_TAG_RELAYS_UPDATE 0x40u         // relay-state bitfield
#define PROTO_TAG_RELAYS_LED_UPDATE 0x41u     // LED-state bitfield
#define PROTO_TAG_SIZE 1u

// Dfu2Request device types (first payload byte, formerly fmt_version).
#define PROTO_DEVICE_TYPE_BULB 0x01u
#define PROTO_DEVICE_TYPE_RELAY 0x02u
#define PROTO_DEVICE_TYPE_PANEL 0x03u

// RelaysUpdate / RelaysLedUpdate: fixed 32-byte bitfield covering the whole
// u8 device id space, LSB-first within each byte.
#define PROTO_RELAYS_BITFIELD_SIZE 32u
#define PROTO_RELAYS_PKT_SIZE (PROTO_TAG_SIZE + PROTO_RELAYS_BITFIELD_SIZE)  // 33

// BulbCommand (0x04). Fixed header: packet_tag + seq(u32) + start(u8) +
// bounds(u8) + cmd(u8) = 8 bytes, followed by a per-command payload.
#define PROTO_BULB_CMD_HDR_SIZE 8u
#define PROTO_CMD_ENTER_DFU 0x00u        // push-DFU: ignored by relay devices
#define PROTO_CMD_SET_CONFIG 0x01u       // u16 key, u8 len, value[len]
#define PROTO_CMD_REBOOT 0x02u
#define PROTO_SETCONFIG_HDR_SIZE 3u      // key(2) + length(1), before the value bytes
#define PROTO_CONFIG_VALUE_MAX 64u

// Dfu2Request (0x05). Fixed header before the variable length-prefixed fields:
// tag + device_type + start + bounds + server_ip(4) + server_port(2) = 10 bytes.
#define PROTO_DFU2_FIXED_SIZE 10u
#define PROTO_DFU2_BUILD_ID_LEN 8u
#define PROTO_DFU2_SSID_MAX 32u
#define PROTO_DFU2_PASS_MAX 63u
// Shared-airtime rule inherited from the ESP8266 bulbs: total vendor-IE body
// must stay <= 70 bytes. RelaysUpdate is 33, so always fine.
#define PROTO_DFU2_MAX_BODY 70u

// ---- parse result -----------------------------------------------------------

typedef struct {
    bool valid;          // frame is a well-formed packet for our protocol

    // RelaysUpdate (0x40) / RelaysLedUpdate (0x41): my bit from the bitfield.
    bool has_relay;      // a RelaysUpdate was decoded; relay_state holds my bit
    bool relay_state;
    bool has_led;        // a RelaysLedUpdate was decoded; led_state holds my bit
    bool led_state;

    // BulbCommand (0x04) fields. is_command is set for any well-formed command
    // regardless of addressing, and seq is then valid — the caller tracks the
    // highest seq seen (anti-replay). reboot_requested / has_config are set
    // only when the command is addressed to my_id. EnterDfuMode is ignored.
    bool     is_command;
    uint32_t seq;
    bool     reboot_requested;
    bool     has_config;
    uint16_t config_key;
    uint8_t  config_len;
    const uint8_t *config_value; // -> value bytes within `frame`; copy before reuse

    // Dfu2Request (0x05) fields. dfu2_requested is set only when the request
    // targets device_type PROTO_DEVICE_TYPE_RELAY AND is addressed to my_id.
    // The pointers alias `frame`; copy before the RX buffer is reused. The
    // caller still gates on build-id comparison.
    bool     dfu2_requested;
    struct {
        const uint8_t *build_id;  uint8_t build_id_len;
        const uint8_t *ssid;      uint8_t ssid_len;
        const uint8_t *pass;      uint8_t pass_len;
        uint8_t  server_ip[4];    // dotted-order octets (a.b.c.d)
        uint16_t server_port;
    } dfu2;
} relay_parse_result_t;

// Parse a raw 802.11 frame (as delivered by the promiscuous RX path).
//
//   frame     : pointer to the first byte of the 802.11 frame (MAC header)
//   frame_len : number of valid bytes at `frame` (may include a 4-byte FCS)
//   my_id     : this device's id (shared bulb/relay id space)
//
// Returns a result with valid=false if the frame is not one of ours. All reads
// are bounds-checked against frame_len; a malformed/truncated frame is
// rejected rather than over-read.
relay_parse_result_t protocol_parse_beacon(const uint8_t *frame, size_t frame_len, uint8_t my_id);

#ifdef __cplusplus
}
#endif

#endif  // RELAY_PROTOCOL_H
