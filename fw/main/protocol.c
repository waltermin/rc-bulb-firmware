// protocol.c — pure parser. No SDK dependencies (see protocol.h).
//
// Frame validation is kept byte-for-byte identical to rc-bulb-firmware's
// protocol.c so both device families accept exactly the same beacons.

#include "protocol.h"

static const relay_parse_result_t INVALID = {
    .valid = false,
    .has_relay = false,
    .relay_state = false,
    .has_led = false,
    .led_state = false,
    .is_command = false,
    .seq = 0,
    .reboot_requested = false,
    .has_config = false,
    .config_key = 0,
    .config_len = 0,
    .config_value = 0,
    .dfu2_requested = false,
    // .dfu2 sub-struct is zero-initialized (pointers NULL, lengths 0).
};

// 802.11 Frame Control (first octet): [subtype:4][type:2][version:2].
// A beacon is subtype=1000, type=00 (management). Masking off the version bits
// yields 0x80.
static inline bool is_beacon(uint8_t fc0) {
    return (fc0 & 0xFCu) == 0x80u;
}

static inline uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ---- per-packet-type parsers -----------------------------------------------
// Each receives `pkt` pointing at the packet tag byte and `pkt_len` bytes
// available from the tag through the end of the IE payload.

// 0x40 / 0x41: [tag, bitfield(32)]. My state is bit my_id of the bitfield,
// LSB-first within each byte.
static relay_parse_result_t parse_relays_update(const uint8_t *pkt, size_t pkt_len,
                                                uint8_t my_id, bool is_led) {
    if (pkt_len < PROTO_RELAYS_PKT_SIZE) {
        return INVALID;
    }

    const uint8_t *bits = pkt + PROTO_TAG_SIZE;
    const bool state = (bits[my_id / 8u] >> (my_id % 8u)) & 1u;

    relay_parse_result_t out = INVALID;
    out.valid = true;
    if (is_led) {
        out.has_led = true;
        out.led_state = state;
    } else {
        out.has_relay = true;
        out.relay_state = state;
    }
    return out;
}

// 0x04 BulbCommand: [tag, seq(u32), start(u8), bounds(u8), cmd(u8), payload...].
// Marks the packet as a command and exposes seq unconditionally (the caller
// does the highest-seq anti-replay gate). Effects are populated only when
// my_id falls in [start, start + bounds]. EnterDfuMode is a bulb-only push-DFU
// concept and is deliberately a no-op here.
static relay_parse_result_t parse_bulb_command(const uint8_t *pkt, size_t pkt_len,
                                               uint8_t my_id) {
    if (pkt_len < PROTO_BULB_CMD_HDR_SIZE) {
        return INVALID;
    }

    const uint32_t seq = read_u32_le(pkt + 1);
    const uint8_t start = pkt[5];
    const uint8_t bounds = pkt[6];
    const uint8_t cmd = pkt[7];

    relay_parse_result_t out = INVALID;
    out.valid = true;
    out.is_command = true;
    out.seq = seq;

    // Addressing: inclusive range [start, start + bounds]; compute in 32-bit to
    // avoid u8 wraparound. Not for us -> valid command (seq tracked), no effect.
    if (my_id < start || (uint32_t)my_id > (uint32_t)start + (uint32_t)bounds) {
        return out;
    }

    switch (cmd) {
        case PROTO_CMD_REBOOT:
            out.reboot_requested = true;
            break;
        case PROTO_CMD_SET_CONFIG: {
            const size_t base = PROTO_BULB_CMD_HDR_SIZE;
            if (pkt_len < base + PROTO_SETCONFIG_HDR_SIZE) {
                break;  // header truncated; ignore effect but keep seq
            }
            const uint16_t key = read_u16_le(pkt + base);
            const uint8_t length = pkt[base + 2];
            if (length > PROTO_CONFIG_VALUE_MAX) {
                break;
            }
            if (pkt_len < base + PROTO_SETCONFIG_HDR_SIZE + length) {
                break;  // value truncated
            }
            out.has_config = true;
            out.config_key = key;
            out.config_len = length;
            out.config_value = pkt + base + PROTO_SETCONFIG_HDR_SIZE;
            break;
        }
        case PROTO_CMD_ENTER_DFU:  // push-DFU: not a relay concept
        default:
            break;  // valid command, no effect
    }

    return out;
}

// 0x05 Dfu2Request: [tag, device_type, start, bounds, ip(4), port(u16),
// build_id(len+n), ssid(len+n), pass(len+n)]. A request for another device
// type is a valid ecosystem frame that simply isn't ours; a request for OUR
// type is then held to full structural validity.
static relay_parse_result_t parse_dfu2_request(const uint8_t *pkt, size_t pkt_len,
                                               uint8_t my_id) {
    if (pkt_len < PROTO_DFU2_FIXED_SIZE) {
        return INVALID;
    }

    relay_parse_result_t out = INVALID;
    out.valid = true;

    if (pkt[1] != PROTO_DEVICE_TYPE_RELAY) {
        return out;  // someone else's rollout (e.g. bulbs); nothing to decode
    }

    const uint8_t start = pkt[2];
    const uint8_t bounds = pkt[3];

    // Addressing: inclusive range [start, start + bounds], 32-bit to avoid u8
    // wraparound. Not for us -> valid frame, no request.
    if (my_id < start || (uint32_t)my_id > (uint32_t)start + (uint32_t)bounds) {
        return out;
    }

    out.dfu2.server_ip[0] = pkt[4];
    out.dfu2.server_ip[1] = pkt[5];
    out.dfu2.server_ip[2] = pkt[6];
    out.dfu2.server_ip[3] = pkt[7];
    out.dfu2.server_port = read_u16_le(pkt + 8);

    // Three length-prefixed variable fields follow the fixed header.
    size_t off = PROTO_DFU2_FIXED_SIZE;  // == 10

    // build_id
    if (off + 1 > pkt_len) return INVALID;
    const uint8_t bl = pkt[off++];
    if (bl == 0 || bl > PROTO_DFU2_BUILD_ID_LEN) return INVALID;
    if (off + bl > pkt_len) return INVALID;
    out.dfu2.build_id = pkt + off;
    out.dfu2.build_id_len = bl;
    off += bl;

    // ssid
    if (off + 1 > pkt_len) return INVALID;
    const uint8_t sl = pkt[off++];
    if (sl == 0 || sl > PROTO_DFU2_SSID_MAX) return INVALID;
    if (off + sl > pkt_len) return INVALID;
    out.dfu2.ssid = pkt + off;
    out.dfu2.ssid_len = sl;
    off += sl;

    // pass (may be empty for an open network)
    if (off + 1 > pkt_len) return INVALID;
    const uint8_t pl = pkt[off++];
    if (pl > PROTO_DFU2_PASS_MAX) return INVALID;
    if (off + pl > pkt_len) return INVALID;
    out.dfu2.pass = pkt + off;
    out.dfu2.pass_len = pl;
    off += pl;

    out.dfu2_requested = true;
    return out;
}

relay_parse_result_t protocol_parse_beacon(const uint8_t *frame, size_t frame_len, uint8_t my_id) {
    if (frame == 0) {
        return INVALID;
    }

    // Need at least a full MAC header + beacon fixed params to reach the IEs.
    const size_t ie_start = IEEE80211_HDR_LEN + BEACON_FIXED_LEN;  // 36
    if (frame_len < ie_start + IEEE80211_IE_HDR_LEN) {
        return INVALID;
    }

    if (!is_beacon(frame[0])) {
        return INVALID;
    }

    // The vendor-specific IE must be the FIRST tagged element.
    const uint8_t *ie = frame + ie_start;
    if (ie[0] != IE_TAG_VENDOR) {
        return INVALID;
    }
    const size_t ie_len = ie[1];  // length of IE data (OUI + payload)

    // IE must contain at least the OUI + a packet tag, and must fit in the frame.
    if (ie_len < VENDOR_OUI_LEN + PROTO_TAG_SIZE) {
        return INVALID;
    }
    const size_t ie_end = ie_start + IEEE80211_IE_HDR_LEN + ie_len;
    if (ie_end > frame_len) {
        return INVALID;
    }

    // Enforce "this must be the ONLY IE". Whatever remains after our IE may only
    // be the optional 4-byte FCS (some drivers include it in the payload, some
    // don't) — anything else means additional IEs are present, so reject.
    const size_t trailing = frame_len - ie_end;
    if (trailing != 0 && trailing != IEEE80211_FCS_LEN) {
        return INVALID;
    }

    // Validate OUI.
    const uint8_t *oui = ie + IEEE80211_IE_HDR_LEN;
    if (oui[0] != VENDOR_OUI_0 || oui[1] != VENDOR_OUI_1 || oui[2] != VENDOR_OUI_2) {
        return INVALID;
    }

    // The packet begins right after the OUI: [packet_tag, ...].
    const uint8_t *pkt = oui + VENDOR_OUI_LEN;
    const size_t pkt_len = ie_len - VENDOR_OUI_LEN;  // >= PROTO_TAG_SIZE (checked above)

    switch (pkt[0]) {
        case PROTO_TAG_RELAYS_UPDATE:
            return parse_relays_update(pkt, pkt_len, my_id, /*is_led=*/false);
        case PROTO_TAG_RELAYS_LED_UPDATE:
            return parse_relays_update(pkt, pkt_len, my_id, /*is_led=*/true);
        case PROTO_TAG_BULB_COMMAND:
            return parse_bulb_command(pkt, pkt_len, my_id);
        case PROTO_TAG_DFU2_REQUEST:
            return parse_dfu2_request(pkt, pkt_len, my_id);
        case PROTO_TAG_LIGHT_UPDATE:
        case PROTO_TAG_PRECISE_LIGHT_UPDATE:
        case PROTO_TAG_LIGHT_UPDATE_V2:
        case PROTO_TAG_RAW_LIGHT_UPDATE: {
            // Bulb color packets: valid ecosystem frames, no effect on a relay.
            relay_parse_result_t out = INVALID;
            out.valid = true;
            return out;
        }
        default:
            return INVALID;  // unknown packet tag
    }
}
