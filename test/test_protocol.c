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

// Build a beacon frame carrying one vendor IE. Returns total length written.
// If good_oui is false a wrong OUI is used. `extra_trailing` bytes of junk are
// appended after our IE (to simulate a second IE) — set to 4 to simulate an FCS.
static size_t build_frame(uint8_t *out, uint8_t version, uint8_t control_flags,
                          uint8_t control_data, const uint8_t *entries,
                          uint8_t entry_count, int good_oui, size_t extra_trailing,
                          uint8_t fc0) {
    size_t n = 0;
    out[n++] = fc0;          // frame control octet 0 (0x80 = beacon)
    out[n++] = 0x00;         // frame control octet 1
    // remainder of 24-byte MAC header + 12-byte fixed params = 34 more bytes
    for (int i = 0; i < 34; i++) out[n++] = 0x00;

    // vendor IE
    size_t pkt_len = PROTO_HEADER_SIZE + (size_t)entry_count * PROTO_ENTRY_SIZE;
    out[n++] = IE_TAG_VENDOR;
    out[n++] = (uint8_t)(VENDOR_OUI_LEN + pkt_len);
    out[n++] = good_oui ? VENDOR_OUI_0 : 0xFF;
    out[n++] = VENDOR_OUI_1;
    out[n++] = VENDOR_OUI_2;
    out[n++] = version;
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

int main(void) {
    uint8_t buf[512];

    // --- valid frame, entry addressed to us ---
    {
        uint8_t entries[] = {
            3, 10, 20, 30, 40, 50,          // not us
            MY_ID, 111, 122, 133, 144, 155, // us
        };
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 2, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("valid + matching entry:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested);
        CHECK(r.r == 111 && r.g == 122 && r.b == 133 && r.ww == 144 && r.cw == 155);
    }

    // --- valid frame but no entry for us ---
    {
        uint8_t entries[] = {3, 10, 20, 30, 40, 50};
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 1, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("valid, no matching entry:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // --- DFU addressed to us ---
    {
        size_t len = build_frame(buf, PROTO_VERSION, PROTO_CTRL_FLAG_DFU, MY_ID, NULL, 0, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("DFU for us:\n");
        CHECK(r.valid);
        CHECK(r.dfu_requested);
    }

    // --- DFU addressed to another bulb ---
    {
        size_t len = build_frame(buf, PROTO_VERSION, PROTO_CTRL_FLAG_DFU, 99, NULL, 0, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("DFU for another id:\n");
        CHECK(r.valid);
        CHECK(!r.dfu_requested);
    }

    // --- trailing 4-byte FCS is allowed ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 1, 1, 4, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("trailing FCS (4 bytes):\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
    }

    // --- a second IE (non-FCS trailing) is rejected ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 1, 1, 6, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("extra IE present:\n");
        CHECK(!r.valid);
    }

    // --- wrong OUI ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 1, 0, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("wrong OUI:\n");
        CHECK(!r.valid);
    }

    // --- wrong protocol version ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, 0x02, 0, 0, entries, 1, 1, 0, 0x80);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("bad version:\n");
        CHECK(!r.valid);
    }

    // --- entry_count over the max ---
    {
        // Hand-build a header claiming 12 entries (but provide none) -> invalid.
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, NULL, 0, 1, 0, 0x80);
        // find entry_count field: ie at offset 36, +2 (tag,len) +3 (oui) +3 (ver,cf,cd) = 44
        buf[36 + 2 + 3 + 3] = 12;
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("entry_count > max:\n");
        CHECK(!r.valid);
    }

    // --- not a beacon (data frame) ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 1, 1, 0, 0x08 /*data*/);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, MY_ID);
        printf("non-beacon frame:\n");
        CHECK(!r.valid);
    }

    // --- truncated frame (cut mid-IE) ---
    {
        uint8_t entries[] = {MY_ID, 1, 2, 3, 4, 5};
        size_t len = build_frame(buf, PROTO_VERSION, 0, 0, entries, 1, 1, 0, 0x80);
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

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
