// controller.c — see controller.h.

#include "controller.h"

void controller_init(controller_t *c, controller_config_t cfg) {
    c->cfg = cfg;
    c->relay_cmd = cfg.default_relay;
    c->led_cmd = cfg.default_led;
    c->relay_heard = false;
    c->led_heard = false;
    c->relay_last_ms = 0;
    c->led_last_ms = 0;
}

void controller_on_relay(controller_t *c, bool state, uint32_t now_ms) {
    c->relay_cmd = state;
    c->relay_heard = true;
    c->relay_last_ms = now_ms;
}

void controller_on_led(controller_t *c, bool state, uint32_t now_ms) {
    c->led_cmd = state;
    c->led_heard = true;
    c->led_last_ms = now_ms;
}

// Unsigned subtraction is wraparound-correct for u32 millisecond clocks as
// long as the real gap is < 2^31 ms (~24 days), far beyond the 15 s window.
static bool timed_out(uint32_t last_ms, uint32_t now_ms, uint32_t timeout_ms) {
    return (uint32_t)(now_ms - last_ms) > timeout_ms;
}

bool controller_relay_out(const controller_t *c, uint32_t now_ms) {
    if (!c->relay_heard || timed_out(c->relay_last_ms, now_ms, c->cfg.timeout_ms)) {
        return c->cfg.default_relay;
    }
    return c->relay_cmd;
}

bool controller_led_out(const controller_t *c, uint32_t now_ms) {
    if (!c->led_heard || timed_out(c->led_last_ms, now_ms, c->cfg.timeout_ms)) {
        return c->cfg.default_led;
    }
    return c->led_cmd;
}
