// test_protocol.c — host unit tests for protocol.c and controller.c.
// Style mirrors rc-bulb-firmware's test harness: build raw frames, assert on
// parse results, exit nonzero on any failure.

#include <stdio.h>
#include <string.h>

#include "../main/protocol.h"
#include "../main/controller.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

// ---- frame builder ----------------------------------------------------------

// Assemble a beacon: MAC header (24) + fixed params (12) + vendor IE wrapping
// `body` (packet tag + payload). Returns total frame length.
static size_t build_beacon(uint8_t *frame, const uint8_t *body, size_t body_len,
                           bool with_fcs, bool extra_ie) {
    memset(frame, 0, 512);
    frame[0] = 0x80;  // beacon
    size_t off = IEEE80211_HDR_LEN + BEACON_FIXED_LEN;  // 36
    frame[off++] = IE_TAG_VENDOR;
    frame[off++] = (uint8_t)(VENDOR_OUI_LEN + body_len);
    frame[off++] = VENDOR_OUI_0;
    frame[off++] = VENDOR_OUI_1;
    frame[off++] = VENDOR_OUI_2;
    memcpy(frame + off, body, body_len);
    off += body_len;
    if (extra_ie) {           // a second IE after ours (must cause rejection)
        frame[off++] = 0x00;  // SSID tag
        frame[off++] = 0x01;
        frame[off++] = 'x';
    }
    if (with_fcs) {
        off += IEEE80211_FCS_LEN;  // 4 bytes of zeros standing in for the FCS
    }
    return off;
}

// Build a RelaysUpdate/RelaysLedUpdate body with only bit `id` set to `state`.
static size_t relays_body(uint8_t *body, uint8_t tag, uint8_t id, bool state) {
    memset(body, 0, PROTO_RELAYS_PKT_SIZE);
    body[0] = tag;
    if (state) {
        body[1 + id / 8] = (uint8_t)(1u << (id % 8));
    }
    return PROTO_RELAYS_PKT_SIZE;
}

// ---- RelaysUpdate / RelaysLedUpdate ------------------------------------------

static void test_relays_update(void) {
    uint8_t frame[512], body[64];

    // my bit set -> relay on
    size_t bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 42, true);
    size_t fl = build_beacon(frame, body, bl, false, false);
    relay_parse_result_t r = protocol_parse_beacon(frame, fl, 42);
    CHECK(r.valid && r.has_relay && r.relay_state);
    CHECK(!r.has_led && !r.is_command && !r.dfu2_requested);

    // my bit clear -> relay off (still has_relay: every packet addresses everyone)
    bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 42, false);
    fl = build_beacon(frame, body, bl, false, false);
    r = protocol_parse_beacon(frame, fl, 42);
    CHECK(r.valid && r.has_relay && !r.relay_state);

    // someone else's bit set, mine clear
    bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 43, true);
    fl = build_beacon(frame, body, bl, false, false);
    r = protocol_parse_beacon(frame, fl, 42);
    CHECK(r.valid && r.has_relay && !r.relay_state);

    // LSB-first bit order: id 0 = byte 0 bit 0; id 9 = byte 1 bit 1;
    // id 255 = byte 31 bit 7
    const uint8_t ids[] = {0, 7, 8, 9, 100, 255};
    for (size_t i = 0; i < sizeof ids; i++) {
        bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, ids[i], true);
        CHECK(body[1 + ids[i] / 8] == (1u << (ids[i] % 8)));
        fl = build_beacon(frame, body, bl, false, false);
        r = protocol_parse_beacon(frame, fl, ids[i]);
        CHECK(r.valid && r.has_relay && r.relay_state);
        // a neighboring id must NOT see its bit set
        r = protocol_parse_beacon(frame, fl, (uint8_t)(ids[i] + 1));
        CHECK(r.valid && r.has_relay && !r.relay_state);
    }

    // FCS-suffixed frame accepted
    bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 5, true);
    fl = build_beacon(frame, body, bl, true, false);
    r = protocol_parse_beacon(frame, fl, 5);
    CHECK(r.valid && r.has_relay && r.relay_state);

    // truncated bitfield rejected (31 bytes instead of 32)
    bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 5, true);
    fl = build_beacon(frame, body, bl - 1, false, false);
    r = protocol_parse_beacon(frame, fl, 5);
    CHECK(!r.valid);

    // extra IE after ours rejected
    bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 5, true);
    fl = build_beacon(frame, body, bl, false, true);
    r = protocol_parse_beacon(frame, fl, 5);
    CHECK(!r.valid);

    // non-beacon rejected
    bl = relays_body(body, PROTO_TAG_RELAYS_UPDATE, 5, true);
    fl = build_beacon(frame, body, bl, false, false);
    frame[0] = 0x40;  // probe request
    r = protocol_parse_beacon(frame, fl, 5);
    CHECK(!r.valid);

    // wrong OUI rejected
    fl = build_beacon(frame, body, bl, false, false);
    frame[38] = 0x99;
    r = protocol_parse_beacon(frame, fl, 5);
    CHECK(!r.valid);

    // LED variant sets has_led, not has_relay
    bl = relays_body(body, PROTO_TAG_RELAYS_LED_UPDATE, 7, true);
    fl = build_beacon(frame, body, bl, false, false);
    r = protocol_parse_beacon(frame, fl, 7);
    CHECK(r.valid && r.has_led && r.led_state && !r.has_relay);
}

// ---- BulbCommand -------------------------------------------------------------

static void test_bulb_command(void) {
    uint8_t frame[512], body[128];

    // Reboot addressed to us
    uint8_t reboot[] = {PROTO_TAG_BULB_COMMAND, 0x0A, 0, 0, 0, /*start*/10, /*bounds*/5, PROTO_CMD_REBOOT};
    size_t fl = build_beacon(frame, reboot, sizeof reboot, false, false);
    relay_parse_result_t r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && r.is_command && r.seq == 10 && r.reboot_requested);

    // Reboot NOT addressed to us: valid command, seq visible, no effect
    r = protocol_parse_beacon(frame, fl, 16);
    CHECK(r.valid && r.is_command && r.seq == 10 && !r.reboot_requested);

    // EnterDfuMode is ignored by relays (valid, no effect)
    uint8_t dfu[] = {PROTO_TAG_BULB_COMMAND, 1, 0, 0, 0, 12, 0, PROTO_CMD_ENTER_DFU};
    fl = build_beacon(frame, dfu, sizeof dfu, false, false);
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && r.is_command && !r.reboot_requested && !r.has_config && !r.dfu2_requested);

    // SetConfig with a 2-byte value
    memcpy(body, (uint8_t[]){PROTO_TAG_BULB_COMMAND, 2, 0, 0, 0, 12, 0, PROTO_CMD_SET_CONFIG,
                             /*key*/0x00, 0x02, /*len*/2, /*value*/0xAB, 0xCD}, 13);
    fl = build_beacon(frame, body, 13, false, false);
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && r.has_config && r.config_key == 0x0200 && r.config_len == 2);
    CHECK(r.config_value && r.config_value[0] == 0xAB && r.config_value[1] == 0xCD);

    // SetConfig with truncated value: no config effect, seq still tracked
    fl = build_beacon(frame, body, 12, false, false);  // one value byte missing
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && r.is_command && !r.has_config && r.seq == 2);
}

// ---- Dfu2Request -------------------------------------------------------------

static size_t dfu2_body(uint8_t *body, uint8_t device_type, uint8_t start, uint8_t bounds) {
    size_t off = 0;
    body[off++] = PROTO_TAG_DFU2_REQUEST;
    body[off++] = device_type;
    body[off++] = start;
    body[off++] = bounds;
    body[off++] = 192; body[off++] = 168; body[off++] = 4; body[off++] = 2;
    body[off++] = 0x05; body[off++] = 0x0D;  // port 3333 LE
    body[off++] = 8;   // build_id len
    for (int i = 0; i < 8; i++) body[off++] = (uint8_t)(0x10 + i);
    body[off++] = 4;   // ssid len
    memcpy(body + off, "test", 4); off += 4;
    body[off++] = 2;   // pass len
    memcpy(body + off, "pw", 2); off += 2;
    return off;
}

static void test_dfu2(void) {
    uint8_t frame[512], body[128];

    // Addressed to us, device_type = relay
    size_t bl = dfu2_body(body, PROTO_DEVICE_TYPE_RELAY, 10, 5);
    size_t fl = build_beacon(frame, body, bl, false, false);
    relay_parse_result_t r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && r.dfu2_requested);
    CHECK(r.dfu2.server_ip[0] == 192 && r.dfu2.server_ip[3] == 2);
    CHECK(r.dfu2.server_port == 3333);
    CHECK(r.dfu2.build_id_len == 8 && r.dfu2.build_id[0] == 0x10);
    CHECK(r.dfu2.ssid_len == 4 && memcmp(r.dfu2.ssid, "test", 4) == 0);
    CHECK(r.dfu2.pass_len == 2 && memcmp(r.dfu2.pass, "pw", 2) == 0);

    // device_type = bulb: valid frame, NOT our rollout
    bl = dfu2_body(body, PROTO_DEVICE_TYPE_BULB, 10, 5);
    fl = build_beacon(frame, body, bl, false, false);
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && !r.dfu2_requested);

    // device_type = panel: same
    bl = dfu2_body(body, PROTO_DEVICE_TYPE_PANEL, 10, 5);
    fl = build_beacon(frame, body, bl, false, false);
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid && !r.dfu2_requested);

    // out of id range: valid, not requested
    bl = dfu2_body(body, PROTO_DEVICE_TYPE_RELAY, 10, 5);
    fl = build_beacon(frame, body, bl, false, false);
    r = protocol_parse_beacon(frame, fl, 40);
    CHECK(r.valid && !r.dfu2_requested);

    // truncated ssid rejected
    bl = dfu2_body(body, PROTO_DEVICE_TYPE_RELAY, 10, 5);
    fl = build_beacon(frame, body, bl - 4, false, false);  // cut into pass/ssid
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(!r.valid);
}

// ---- bulb color packets are valid no-ops -------------------------------------

static void test_bulb_packets_ignored(void) {
    uint8_t frame[512], body_big[64];
    // LightUpdateV2 with one entry for id 12
    uint8_t body[] = {PROTO_TAG_LIGHT_UPDATE_V2, 1, /*entry*/12, 255, 0, 0, 0, 0};
    size_t fl = build_beacon(frame, body, sizeof body, false, false);
    relay_parse_result_t r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid);
    CHECK(!r.has_relay && !r.has_led && !r.is_command && !r.dfu2_requested);

    // RawLightUpdate (0x06, bulb-only): tag + id + 5*f32 + 5*u8 = 27 bytes, all
    // bits set so a misparse as a relay bitfield would turn the relay on.
    uint8_t raw[27];
    memset(raw, 0xFF, sizeof raw);
    raw[0] = PROTO_TAG_RAW_LIGHT_UPDATE;
    fl = build_beacon(frame, raw, sizeof raw, false, false);
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(r.valid);
    CHECK(!r.has_relay && !r.has_led && !r.is_command && !r.dfu2_requested);

    // A full 33-byte bitfield under the bulb's 0x06 tag must not drive the relay
    // either: relay packets live at 0x40/0x41, never at a bulb tag.
    relays_body(body_big, PROTO_TAG_RAW_LIGHT_UPDATE, 12, true);
    fl = build_beacon(frame, body_big, PROTO_RELAYS_PKT_SIZE, false, false);
    r = protocol_parse_beacon(frame, fl, 12);
    CHECK(!r.has_relay && !r.has_led);
}

// ---- controller ---------------------------------------------------------------

static void test_controller(void) {
    controller_t c;
    controller_config_t cfg = {.default_relay = false, .default_led = true, .timeout_ms = 15000};
    controller_init(&c, cfg);

    // boot: defaults apply before anything is heard
    CHECK(controller_relay_out(&c, 0) == false);
    CHECK(controller_led_out(&c, 0) == true);

    // relay packet arrives: follow it; LED still default
    controller_on_relay(&c, true, 1000);
    CHECK(controller_relay_out(&c, 1001) == true);
    CHECK(controller_led_out(&c, 1001) == true);

    // led packet arrives: follow it
    controller_on_led(&c, false, 2000);
    CHECK(controller_led_out(&c, 2001) == false);

    // independent timers: relay falls back at 1000+15001, LED at 2000+15001
    CHECK(controller_relay_out(&c, 16000) == true);    // 15000ms exactly: not yet
    CHECK(controller_relay_out(&c, 16002) == false);   // relay timed out -> default off
    CHECK(controller_led_out(&c, 16002) == false);     // LED not yet timed out
    CHECK(controller_led_out(&c, 17002) == true);      // LED timed out -> default on

    // fresh packet after fallback re-arms
    controller_on_relay(&c, true, 20000);
    CHECK(controller_relay_out(&c, 20001) == true);

    // u32 wraparound: last packet just before wrap, query just after
    controller_on_relay(&c, true, 0xFFFFFF00u);
    CHECK(controller_relay_out(&c, 0x00000100u) == true);   // ~512ms gap, no timeout
    CHECK(controller_relay_out(&c, 0x00004000u) == false);  // >15s past wrap, default
}

int main(void) {
    test_relays_update();
    test_bulb_command();
    test_dfu2();
    test_bulb_packets_ignored();
    test_controller();

    if (failures == 0) {
        printf("all tests passed\n");
        return 0;
    }
    printf("%d failure(s)\n", failures);
    return 1;
}
