// airtest.c — over-the-air functional test for a finished relay switch.
//
// Broadcasts RelaysUpdate (0x40) and RelaysLedUpdate (0x41) with only the
// target's bit driven, through a fixed pattern a person can watch and hear.
// Runs as a background task and never prints, so it can overlap a flash that
// is using the USB <-> UART bridge.
//
// RelaysUpdate is full state: every relay device in range follows its own bit,
// so the others are held OFF (their default) for the duration.

#include "airtest.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radio.h"

#define TAG_RELAYS     0x40
#define TAG_RELAYS_LED 0x41
#define FRAME_MS       100  // each test step re-sends both packets this often
#define HOLD_FRAME_MS  50   // held states go out this often...
#define HOLD_REPEAT    3    // ...with relay/LED bitfields repeated this many times

typedef struct {
    uint16_t ms;
    uint8_t relay, led;
} step_t;

// Keep in sync with the timeline printed by tools/bkjig2.py `test`.
static const step_t STEPS[] = {
    {2000, 1, 0}, {2000, 0, 0}, {2000, 1, 0}, {2000, 0, 0},                              // relay x2
    {3000, 1, 1},                                                                        // both on
    {2000, 0, 0},                                                                        // both off
};

static volatile bool s_running;
static uint8_t s_id;

// HOLD state (see "hold" below); declared here because the test task restores
// the hold channel when it finishes.
static uint8_t s_hold_relay[32], s_hold_led[32];
static bool s_hold_relay_on, s_hold_led_on;
static uint8_t s_hold_ch = 4;
static bool s_hold_task;
#define RAW_SLOTS 8
static uint8_t s_raw[RAW_SLOTS][70];  // any packet body (tag + payload), e.g. 0x03 LightUpdateV2
static size_t  s_raw_len[RAW_SLOTS];  // 0 = slot empty
static bool hold_active(void);
static void hold_ensure_task(uint8_t channel);

static void send_state(uint8_t tag, uint8_t on) {
    uint8_t body[33] = {tag};
    if (on) body[1 + s_id / 8] = (uint8_t)(1u << (s_id % 8));
    radio_send_beacon(body, sizeof body);
}

static void airtest_task(void *arg) {
    for (size_t i = 0; i < sizeof STEPS / sizeof STEPS[0] && s_running; i++) {
        for (uint16_t t = 0; t < STEPS[i].ms && s_running; t += FRAME_MS) {
            send_state(TAG_RELAYS, STEPS[i].relay);
            send_state(TAG_RELAYS_LED, STEPS[i].led);
            vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
        }
    }
    s_running = false;
    if (hold_active()) radio_start_sta(s_hold_ch);
    vTaskDelete(NULL);
}

bool airtest_start(uint8_t id, uint8_t channel) {
    if (s_running) return false;
    s_id = id;
    radio_start_sta(channel);
    s_running = true;
    xTaskCreate(airtest_task, "airtest", 6144, NULL, 5, NULL);
    return true;
}

bool airtest_running(void) { return s_running; }

// ---- hold: keep a relay/LED state broadcast until told to stop -------------
// Devices fall back to their defaults after ~15 s without packets, so a held
// state has to be re-sent continuously. Paused while an air test runs.

static bool hold_active(void) {
    if (s_hold_relay_on || s_hold_led_on) return true;
    for (int i = 0; i < RAW_SLOTS; i++) {
        if (s_raw_len[i]) return true;
    }
    return false;
}

static void hold_task(void *arg) {
    while (hold_active()) {
        if (!s_running) {
            uint8_t body[33];
            for (int r = 0; r < HOLD_REPEAT; r++) {
                if (s_hold_relay_on) {
                    body[0] = TAG_RELAYS;
                    memcpy(body + 1, s_hold_relay, 32);
                    radio_send_beacon(body, sizeof body);
                }
                if (s_hold_led_on) {
                    body[0] = TAG_RELAYS_LED;
                    memcpy(body + 1, s_hold_led, 32);
                    radio_send_beacon(body, sizeof body);
                }
            }
            for (int i = 0; i < RAW_SLOTS; i++) {
                if (s_raw_len[i]) radio_send_beacon(s_raw[i], s_raw_len[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(HOLD_FRAME_MS));
    }
    s_hold_task = false;
    vTaskDelete(NULL);
}

void airtest_hold(bool led, const uint8_t bits[32], uint8_t channel) {
    memcpy(led ? s_hold_led : s_hold_relay, bits, 32);
    if (led) s_hold_led_on = true; else s_hold_relay_on = true;
    hold_ensure_task(channel);
}

static void hold_ensure_task(uint8_t channel) {
    s_hold_ch = channel;
    radio_start_sta(channel);
    if (!s_hold_task) {
        s_hold_task = true;
        xTaskCreate(hold_task, "hold", 6144, NULL, 5, NULL);
    }
}

bool airtest_hold_raw(int slot, const uint8_t *body, size_t len, uint8_t channel) {
    if (slot < 0 || slot >= RAW_SLOTS || len == 0 || len > sizeof s_raw[0]) return false;
    s_raw_len[slot] = 0;  // the task never sends a half-written slot
    memcpy(s_raw[slot], body, len);
    s_raw_len[slot] = len;
    hold_ensure_task(channel);
    return true;
}

void airtest_hold_clear(int what) {
    if (what == HOLD_CLEAR_RELAY || what == HOLD_CLEAR_ALL) s_hold_relay_on = false;
    if (what == HOLD_CLEAR_LED || what == HOLD_CLEAR_ALL) s_hold_led_on = false;
    if (what == HOLD_CLEAR_RAW || what == HOLD_CLEAR_ALL) {
        for (int i = 0; i < RAW_SLOTS; i++) s_raw_len[i] = 0;
    }
}

void airtest_hold_off(void) { airtest_hold_clear(HOLD_CLEAR_ALL); }

bool airtest_holding(void) { return hold_active(); }

void airtest_stop(void) { s_running = false; }

uint32_t airtest_duration_ms(void) {
    uint32_t ms = 0;
    for (size_t i = 0; i < sizeof STEPS / sizeof STEPS[0]; i++) ms += STEPS[i].ms;
    return ms;
}
