// protocol.c — pure parser. No SDK dependencies (see protocol.h).

#include "protocol.h"

static const bulb_parse_result_t INVALID = {
    .valid = false,
    .dfu_requested = false,
    .has_entry = false,
    .r = 0, .g = 0, .b = 0, .ww = 0, .cw = 0,
};

// 802.11 Frame Control (first octet): [subtype:4][type:2][version:2].
// A beacon is subtype=1000, type=00 (management). Masking off the version bits
// yields 0x80. This lets us ignore the protocol-version field while still
// requiring management/beacon.
static inline bool is_beacon(uint8_t fc0) {
    return (fc0 & 0xFCu) == 0x80u;
}

bulb_parse_result_t protocol_parse_beacon(const uint8_t *frame, size_t frame_len, uint8_t my_id) {
    if (frame == NULL) {
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

    // IE must contain at least the OUI + a full LightUpdatePacket header, and
    // must fit within the frame.
    if (ie_len < VENDOR_OUI_LEN + PROTO_HEADER_SIZE) {
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

    // LightUpdatePacket begins right after the OUI.
    const uint8_t *pkt = oui + VENDOR_OUI_LEN;
    const size_t pkt_len = ie_len - VENDOR_OUI_LEN;  // >= PROTO_HEADER_SIZE (checked above)

    const uint8_t version = pkt[0];
    const uint8_t control_flags = pkt[1];
    const uint8_t control_data = pkt[2];
    const uint8_t entry_count = pkt[3];

    if (version != PROTO_VERSION) {
        return INVALID;
    }
    if (entry_count > PROTO_MAX_ENTRIES) {
        return INVALID;
    }

    // The declared entries must fit exactly-or-within the packet payload.
    const size_t need = PROTO_HEADER_SIZE + (size_t)entry_count * PROTO_ENTRY_SIZE;
    if (need > pkt_len) {
        return INVALID;
    }

    bulb_parse_result_t out = {
        .valid = true,
        .dfu_requested = (control_flags == PROTO_CTRL_FLAG_DFU) && (control_data == my_id),
        .has_entry = false,
        .r = 0, .g = 0, .b = 0, .ww = 0, .cw = 0,
    };

    // Scan entries for the first one addressed to us.
    const uint8_t *entries = pkt + PROTO_HEADER_SIZE;
    for (uint8_t i = 0; i < entry_count; i++) {
        const uint8_t *e = entries + (size_t)i * PROTO_ENTRY_SIZE;
        if (e[0] == my_id) {
            out.has_entry = true;
            out.r = e[1];
            out.g = e[2];
            out.b = e[3];
            out.ww = e[4];
            out.cw = e[5];
            break;
        }
    }

    return out;
}
