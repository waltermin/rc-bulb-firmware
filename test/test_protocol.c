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

int main(void) {
    uint8_t buf[512];

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

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
