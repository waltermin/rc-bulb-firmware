// controller.h — relay/LED state machine: broadcast state + dual fallback.
//
// Pure logic, no SDK dependencies (host-testable like protocol.*). The
// platform layer feeds it parsed packets and a millisecond clock; it answers
// "what should the relay and LED be right now".
//
// Behavior (spec.txt "Relay device behavior"):
//  - At power-on both outputs are their configured DEFAULTs.
//  - A RelaysUpdate sets the relay and re-arms the relay fallback timer; a
//    RelaysLedUpdate does the same for the LED. The timers are independent.
//  - If a timer sees no packet of its type for timeout_ms, that output reverts
//    to its DEFAULT until the next packet of that type arrives.

#ifndef RELAY_CONTROLLER_H
#define RELAY_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool default_relay;
    bool default_led;
    uint32_t timeout_ms;   // fallback window per packet type
} controller_config_t;

typedef struct {
    controller_config_t cfg;
    bool relay_cmd;         // last commanded relay state
    bool led_cmd;           // last commanded LED state
    bool relay_heard;       // any RelaysUpdate heard since boot
    bool led_heard;         // any RelaysLedUpdate heard since boot
    uint32_t relay_last_ms; // time of last RelaysUpdate
    uint32_t led_last_ms;   // time of last RelaysLedUpdate
} controller_t;

void controller_init(controller_t *c, controller_config_t cfg);

// Feed parsed packet effects (call only when has_relay / has_led was set).
void controller_on_relay(controller_t *c, bool state, uint32_t now_ms);
void controller_on_led(controller_t *c, bool state, uint32_t now_ms);

// Current desired outputs. Pure function of stored state + now; safe to call
// at any rate. Handles u32 millisecond wraparound (unsigned subtraction).
bool controller_relay_out(const controller_t *c, uint32_t now_ms);
bool controller_led_out(const controller_t *c, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif  // RELAY_CONTROLLER_H
