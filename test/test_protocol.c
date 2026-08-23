// test_protocol.c — host unit tests for the pure protocol parser.
// Build & run:  cc -I../main test_protocol.c ../main/protocol.c -o test && ./test

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"

#define MY_ID 7

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        g_checks++;                                                       \
        if (!(cond)) {                                                    \
            g_fails++;                                                    \
            printf("  FAIL: %s (line %d)\n", #cond, __LINE__);            \
        }                                                                 \
    } while (0)

// Float compare with a tolerance that covers 1/255 quantization error.
#define FEQ(a, b) (((a) - (b) < 0.001f) && ((b) - (a) < 0.001f))

// Build a beacon frame carrying one vendor IE holding a LightUpdate (0x01)
// packet. Returns total length written. If good_oui is false a wrong OUI is
// used. `extra_trailing` bytes of junk are appended after our IE (to simulate a
// second IE) — set to 4 to simulate an FCS. `tag` is the packet tag byte.
static size_t build_frame(uint8_t *out, uint8_t tag, uint8_t control_flags,
                          uint8_t control_data, const uint8_t *entries,
                          uint8_t entry_count, int good_oui, size_t extra_trailing,
                          uint8_t fc0) {
    size_t n = 0;
    out[n++] = fc0;          // frame control octet 0 (0x80 = beacon)
    out[n++] = 0x00;         // frame control octet 1
    // remainder of 24-byte MAC header + 12-byte fixed params = 34 more bytes
    for (int i = 0; i < 34; i++) out[n++] = 0x00;

    // vendor IE
    size_t pkt_len = PROTO_LIGHT_UPDATE_HDR_SIZE + (size_t)entry_count * PROTO_ENTRY_SIZE;
    out[n++] = IE_TAG_VENDOR;
    out[n++] = (uint8_t)(VENDOR_OUI_LEN + pkt_len);
    out[n++] = good_oui ? VENDOR_OUI_0 : 0xFF;
    out[n++] = VENDOR_OUI_1;
    out[n++] = VENDOR_OUI_2;
    out[n++] = tag;
    out[n++] = control_flags;
    out[n++] = control_data;
    out[n++] = entry_count;
    for (size_t i = 0; i < (size_t)entry_count * PROTO_ENTRY_SIZE; i++) {
        out[n++] = entries[i];
    }
    for (size_t i = 0; i < extra_trailing; i++) {
        out[n++] = 0xAB;
    }
    return n;
}

// Build a beacon carrying a PreciseLightUpdate (0x02) packet for `bulb_id` with
// five float channels. Returns total length written.
static size_t build_precise_frame(uint8_t *out, uint8_t bulb_id,
                                  const float colors[5]) {
    size_t n = 0;
    out[n++] = 0x80;  // beacon
    out[n++] = 0x00;
    for (int i = 0; i < 34; i++) out[n++] = 0x00;

    out[n++] = IE_TAG_VENDOR;
    out[n++] = (uint8_t)(VENDOR_OUI_LEN + PROTO_PRECISE_SIZE);
    out[n++] = VENDOR_OUI_0;
    out[n++] = VENDOR_OUI_1;
    out[n++] = VENDOR_OUI_2;
    out[n++] = PROTO_TAG_PRECISE_LIGHT_UPDATE;
    out[n++] = bulb_id;
    for (int i = 0; i < 5; i++) {
        // little-endian IEEE-754
        uint8_t bytes[4];
        memcpy(bytes, &colors[i], 4);
        out[n++] = bytes[0];
        out[n++] = bytes[1];
        out[n++] = bytes[2];
        out[n++] = bytes[3];
    }
    return n;
}

// Wrap a raw packet (bytes starting at the packet tag) in a beacon frame with
// our vendor IE as the only IE. Returns total frame length.
static size_t wrap_beacon(uint8_t *out, const uint8_t *pkt, size_t pkt_len) {
    size_t n = 0;
    out[n++] = 0x80;  // beacon
    out[n++] = 0x00;
    for (int i = 0; i < 34; i++) out[n++] = 0x00;  // rest of MAC hdr + fixed params
    out[n++] = IE_TAG_VENDOR;
    out[n++] = (uint8_t)(VENDOR_OUI_LEN + pkt_len);
    out[n++] = VENDOR_OUI_0;
    out[n++] = VENDOR_OUI_1;
    out[n++] = VENDOR_OUI_2;
    for (size_t i = 0; i < pkt_len; i++) out[n++] = pkt[i];
    return n;
}

static void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// Build a beacon carrying a LightUpdateV2 (0x03) packet: [tag, entry_count, ...].
static size_t build_v2_frame(uint8_t *out, const uint8_t *entries, uint8_t entry_count) {
    uint8_t pkt[PROTO_LIGHT_UPDATE_V2_HDR_SIZE + PROTO_MAX_ENTRIES * PROTO_ENTRY_SIZE];
    size_t m = 0;
    pkt[m++] = PROTO_TAG_LIGHT_UPDATE_V2;
    pkt[m++] = entry_count;
    for (size_t i = 0; i < (size_t)entry_count * PROTO_ENTRY_SIZE; i++) pkt[m++] = entries[i];
    return wrap_beacon(out, pkt, m);
}

// Build a beacon carrying a BulbCommand (0x04) with the given fixed-header fields
// and raw payload bytes following cmd.
static size_t build_cmd_frame(uint8_t *out, uint32_t seq, uint8_t start, uint8_t bounds,
                              uint8_t cmd, const uint8_t *payload, size_t payload_len) {
    uint8_t pkt[PROTO_BULB_CMD_HDR_SIZE + PROTO_SETCONFIG_HDR_SIZE + PROTO_CONFIG_VALUE_MAX];
    size_t m = 0;
    pkt[m++] = PROTO_TAG_BULB_COMMAND;
    put_u32_le(&pkt[m], seq); m += 4;
    pkt[m++] = start;
    pkt[m++] = bounds;
    pkt[m++] = cmd;
    for (size_t i = 0; i < payload_len; i++) pkt[m++] = payload[i];
    return wrap_beacon(out, pkt, m);
}

int main(void) {
    uint8_t buf[512];

#if PROTO_ENABLE_LEGACY_LIGHT_UPDATE
    // --- valid LightUpdate frame, entry addressed to us ---
    {
        uint8_t entries[] = {
            3, 10, 20, 30, 40, 50,          // not us
            MY_ID, 111, 122, 133, 144, 155, // us
        };
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 2, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("valid + matching entry:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested);
        CHECK(FEQ(r.r, 111 / 255.0f) && FEQ(r.g, 122 / 255.0f) && FEQ(r.b, 133 / 255.0f) &&
              FEQ(r.ww, 144 / 255.0f) && FEQ(r.cw, 155 / 255.0f));
    }

    // --- valid frame but no entry for us ---
    {
        uint8_t entries[] = {3, 10, 20, 30, 40, 50};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("valid, no matching entry:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // --- DFU addressed to us ---
    {
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, PROTO_CTRL_FLAG_DFU, MY_ID, NULL, 0, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("DFU for us:\n");
        CHECK(r.valid);
        CHECK(r.dfu_requested);
    }

    // --- DFU addressed to another bulb ---
    {
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, PROTO_CTRL_FLAG_DFU, 99, NULL, 0, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("DFU for another id:\n");
        CHECK(r.valid);
        CHECK(!r.dfu_requested);
    }

    // --- trailing 4-byte FCS is allowed ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 1, 4, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("trailing FCS (4 bytes):\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
    }
#else
    // --- legacy 0x01 is ignored when disabled ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("legacy 0x01 ignored (disabled):\n");
        CHECK(!r.valid);
    }
#endif  // PROTO_ENABLE_LEGACY_LIGHT_UPDATE

    // --- a second IE (non-FCS trailing) is rejected ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 1, 6, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("extra IE present:\n");
        CHECK(!r.valid);
    }

    // --- wrong OUI ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 0, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("wrong OUI:\n");
        CHECK(!r.valid);
    }

    // --- unknown packet tag ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, 0x7F, 0, 0, entries, 1, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("unknown packet tag:\n");
        CHECK(!r.valid);
    }

    // --- entry_count over the max ---
    {
        // Hand-build a header claiming 12 entries (but provide none) -> invalid.
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, NULL, 0, 1, 0, 0x80);
        // find entry_count field: ie at offset 36, +2 (tag,len) +3 (oui) +3 (tag,cf,cd) = 44
        buf[36 + 2 + 3 + 3] = 12;
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("entry_count > max:\n");
        CHECK(!r.valid);
    }

    // --- not a beacon (data frame) ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 1, 0, 0x08 /*data*/);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("non-beacon frame:\n");
        CHECK(!r.valid);
    }

    // --- truncated frame (cut mid-IE) ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_TAG_LIGHT_UPDATE, 0, 0, entries, 1, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len - 3, MY_ID);
        printf("truncated frame:\n");
        CHECK(!r.valid);
    }

    // --- too short to even reach IEs ---
    {
        bulb_parse_result_t r = protocol_parse_beacon(buf, 10, MY_ID);
        printf("runt frame:\n");
        CHECK(!r.valid);
    }

    // --- PreciseLightUpdate addressed to us ---
    {
        float colors[5] = {1.0f, 0.5f, 0.0f, 0.25f, 0.75f};
        size_t len = build_precise_frame(buf, MY_ID, colors);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("precise for us:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested);
        CHECK(FEQ(r.r, 1.0f) && FEQ(r.g, 0.5f) && FEQ(r.b, 0.0f) &&
              FEQ(r.ww, 0.25f) && FEQ(r.cw, 0.75f));
    }

    // --- PreciseLightUpdate addressed to another bulb ---
    {
        float colors[5] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        size_t len = build_precise_frame(buf, 99, colors);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("precise for another id:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // --- PreciseLightUpdate out-of-range floats are clamped to [0,1] ---
    {
        float colors[5] = {2.0f, -1.0f, 0.5f, 100.0f, -0.001f};
        size_t len = build_precise_frame(buf, MY_ID, colors);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("precise clamp:\n");
        CHECK(r.valid && r.has_entry);
        CHECK(FEQ(r.r, 1.0f) && FEQ(r.g, 0.0f) && FEQ(r.b, 0.5f) &&
              FEQ(r.ww, 1.0f) && FEQ(r.cw, 0.0f));
    }

    // --- truncated PreciseLightUpdate (one byte short) is rejected ---
    {
        float colors[5] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        size_t len = build_precise_frame(buf, MY_ID, colors);
        // Drop the final color byte and shrink the IE length to match, so the
        // framing is well-formed but the precise payload is one byte short.
        buf[37] -= 1;  // IE length field (offset 36 tag, 37 length)
        bulb_parse_result_t r = protocol_parse_beacon(buf, len - 1, MY_ID);
        printf("truncated precise:\n");
        CHECK(!r.valid);
    }

    // --- LightUpdateV2 (0x03) addressed to us ---
    {
        uint8_t entries[] = {
            3, 10, 20, 30, 40, 50,           // not us
            MY_ID, 11, 22, 33, 44, 55,       // us
        };
        size_t len = build_v2_frame(buf, entries, 2);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("v2 + matching entry:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested && !r.is_command);
        CHECK(FEQ(r.r, 11 / 255.0f) && FEQ(r.g, 22 / 255.0f) && FEQ(r.b, 33 / 255.0f) &&
              FEQ(r.ww, 44 / 255.0f) && FEQ(r.cw, 55 / 255.0f));
    }

    // --- LightUpdateV2 with no entry for us ---
    {
        uint8_t entries[] = {3, 10, 20, 30, 40, 50};
        size_t len = build_v2_frame(buf, entries, 1);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("v2, no matching entry:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // --- LightUpdateV2 over the entry max ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_v2_frame(buf, entries, 1);
        buf[36 + 2 + 3 + 1] = 12;  // entry_count field: ie(36) + tag/len(2) + oui(3) + tag(1)
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("v2 entry_count > max:\n");
        CHECK(!r.valid);
    }

    // --- BulbCommand EnterDfu (0x04/0x00) addressed to us (bounds 0) ---
    {
        size_t len = build_cmd_frame(buf, 100, MY_ID, 0, PROTO_CMD_ENTER_DFU, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command EnterDfu for us:\n");
        CHECK(r.valid);
        CHECK(r.is_command);
        CHECK(r.seq == 100);
        CHECK(r.dfu_requested);
        CHECK(!r.has_config);
    }

    // --- BulbCommand EnterDfu addressed by an inclusive range covering us ---
    {
        size_t len = build_cmd_frame(buf, 101, MY_ID - 2, 5, PROTO_CMD_ENTER_DFU, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command EnterDfu range covers us:\n");
        CHECK(r.valid && r.is_command);
        CHECK(r.dfu_requested);
    }

    // --- BulbCommand not addressed to us: valid + seq tracked, but no effect ---
    {
        size_t len = build_cmd_frame(buf, 102, MY_ID + 1, 0, PROTO_CMD_ENTER_DFU, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command not for us:\n");
        CHECK(r.valid);
        CHECK(r.is_command);
        CHECK(r.seq == 102);
        CHECK(!r.dfu_requested);
        CHECK(!r.has_config);
    }

    // --- BulbCommand SetConfig (0x04/0x01) addressed to us ---
    {
        uint8_t payload[PROTO_SETCONFIG_HDR_SIZE + 4];
        payload[0] = 0x31; payload[1] = 0x00;  // key = 0x0031 (little-endian)
        payload[2] = 4;                          // length
        payload[3] = 0xDE; payload[4] = 0xAD; payload[5] = 0xBE; payload[6] = 0xEF;
        size_t len = build_cmd_frame(buf, 103, MY_ID, 0, PROTO_CMD_SET_CONFIG,
                                     payload, sizeof(payload));
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command SetConfig for us:\n");
        CHECK(r.valid && r.is_command);
        CHECK(r.seq == 103);
        CHECK(!r.dfu_requested);
        CHECK(r.has_config);
        CHECK(r.config_key == 0x0031);
        CHECK(r.config_len == 4);
        CHECK(r.config_value != NULL);
        CHECK(r.config_value[0] == 0xDE && r.config_value[3] == 0xEF);
    }

    // --- SetConfig with a truncated value is rejected (effect only) ---
    {
        uint8_t payload[PROTO_SETCONFIG_HDR_SIZE + 2];
        payload[0] = 0x31; payload[1] = 0x00;
        payload[2] = 4;                          // claims 4 value bytes...
        payload[3] = 0x01; payload[4] = 0x02;    // ...but only 2 present
        size_t len = build_cmd_frame(buf, 104, MY_ID, 0, PROTO_CMD_SET_CONFIG,
                                     payload, sizeof(payload));
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command SetConfig truncated value:\n");
        CHECK(r.valid && r.is_command);  // still a valid command (seq tracked)
        CHECK(!r.has_config);            // but no config effect
    }

    // --- SetConfig value longer than we accept is rejected ---
    {
        uint8_t payload[PROTO_SETCONFIG_HDR_SIZE];
        payload[0] = 0x31; payload[1] = 0x00;
        payload[2] = PROTO_CONFIG_VALUE_MAX + 1;  // too long
        size_t len = build_cmd_frame(buf, 105, MY_ID, 0, PROTO_CMD_SET_CONFIG,
                                     payload, sizeof(payload));
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command SetConfig over-long value:\n");
        CHECK(r.valid && r.is_command);
        CHECK(!r.has_config);
    }

    // --- BulbCommand Reboot (0x04/0x02) addressed to us (bounds 0) ---
    {
        size_t len = build_cmd_frame(buf, 107, MY_ID, 0, PROTO_CMD_REBOOT, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command Reboot for us:\n");
        CHECK(r.valid);
        CHECK(r.is_command);
        CHECK(r.seq == 107);
        CHECK(r.reboot_requested);
        CHECK(!r.dfu_requested);
        CHECK(!r.has_config);
    }

    // --- BulbCommand Reboot addressed by a range covering us ---
    {
        size_t len = build_cmd_frame(buf, 108, MY_ID - 1, 3, PROTO_CMD_REBOOT, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command Reboot range covers us:\n");
        CHECK(r.valid && r.is_command);
        CHECK(r.reboot_requested);
    }

    // --- BulbCommand Reboot not addressed to us: seq tracked, no effect ---
    {
        size_t len = build_cmd_frame(buf, 109, MY_ID + 1, 0, PROTO_CMD_REBOOT, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command Reboot not for us:\n");
        CHECK(r.valid && r.is_command && r.seq == 109);
        CHECK(!r.reboot_requested);
    }

    // --- unknown command: valid + seq tracked, no effect ---
    {
        size_t len = build_cmd_frame(buf, 106, MY_ID, 0, 0x7F, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("unknown command:\n");
        CHECK(r.valid && r.is_command && r.seq == 106);
        CHECK(!r.dfu_requested && !r.has_config);
    }

    // --- BulbCommand with a truncated fixed header is rejected ---
    {
        uint8_t pkt[1] = {PROTO_TAG_BULB_COMMAND};  // tag only, no seq/addr/cmd
        size_t len = wrap_beacon(buf, pkt, sizeof(pkt));
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("command truncated header:\n");
        CHECK(!r.valid);
    }

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
