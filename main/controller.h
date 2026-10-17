// controller.h — owns light state: applies incoming colors, runs the fallback
// timeout, and triggers DFU mode. Fed by the sniffer RX path.

#ifndef BULB_CONTROLLER_H
#define BULB_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "dfu2.h"  // dfu2_params_t

#ifdef __cplusplus
extern "C" {
#endif

// Create the controller queue + task. The bulb id (used for DFU logging/handoff)
// is read live from the config store when needed, so no id is passed in here.
void controller_start(void);

// Post a new color addressed to us, as normalized duty cycles in [0.0, 1.0]
// (the protocol parser has already scaled/clamped both packet types into this
// range). Safe to call from the promiscuous RX callback context. Non-blocking;
// drops if the queue is momentarily full (returns false).
bool controller_notify_entry(float r, float g, float b, float ww, float cw);

// Post a RAW color addressed to us (0x06 RawLightUpdate): duty cycles in [0,1]
// applied WITHOUT the gamma/perceptual curve or max-power cap (so they can
// overdrive), each channel with its own PWM period as a log2(ticks) exponent
// (clamped to the firmware range by the PWM layer). Resets the fallback timer like
// a normal color. Safe from the RX callback; non-blocking; drops if the queue is
// full. Color and period args are both in the fixed order r, g, b, ww, cw.
bool controller_notify_raw_entry(float r, float g, float b, float ww, float cw,
                                 uint8_t period_r, uint8_t period_g, uint8_t period_b,
                                 uint8_t period_ww, uint8_t period_cw);

// Post a Lemtone value addressed to us (0x07 LemtoneUpdate), already decoded from the
// wire: OKLab chromaticity a, b at L = 1, lumens (0 = off), fidelity 0..1. The
// controller task runs the Lemtone solver and drives the PWM from its duties and
// periods. Resets the fallback timer. Safe from the RX callback; non-blocking.
// Returns false if the queue was full and the value was dropped. Only acted on by
// devices with LEMTONE_ENABLED (the sniffer does not post it otherwise).
bool controller_notify_lemtone(float a, float b, float lm, float fidelity);

// Post a keep-alive: the light we are showing was just re-sent unchanged (the RX path
// drops such repeats before they cost a solve). Resets the fallback timer only; ignored
// once the controller has fallen back to the default. Safe from the RX callback.
void controller_notify_keepalive(void);

// Counters for the Lemtone path (diagnostics / self-test). Written by the controller
// task only; a reader may see a slightly stale snapshot.
typedef struct {
    uint32_t applies;          // Lemtone values solved and applied
    uint32_t fallbacks;        // times the fallback timeout reverted to the default
    uint8_t  last_mode;        // lemtone_mode_t of the last solve
    uint16_t last_evals;
    uint32_t last_solve_us;    // lemtone_compute only
    uint32_t max_solve_us;
    uint32_t last_pwm_us;      // pwm_output_set_raw (schedule compile + publish)
    uint32_t max_pwm_us;
    uint32_t min_stack_free;   // controller task stack high-water mark, bytes
} controller_lemtone_stats_t;
void controller_get_lemtone_stats(controller_lemtone_stats_t *out);

// Post a DFU2 (pull-based OTA) request with the parameters copied out of a
// Dfu2Request beacon. Applied on the controller task, which hands off to the DFU2
// task and stands down. Safe to call from the promiscuous RX callback context;
// `params` is copied. Non-blocking; drops if the queue is full (a re-broadcast
// will retrigger).
void controller_notify_dfu2(const dfu2_params_t *params);

// Post a reboot request (from a Reboot BulbCommand). Applied on the controller
// task, which calls esp_restart(). Safe to call from the promiscuous RX callback
// context. Non-blocking; drops if the queue is full (a retransmit will retrigger).
void controller_notify_reboot(void);

// Post a config write (from a SetConfig BulbCommand): `key` is the 16-bit wire
// tag, `value`/`len` the raw bytes. Applied on the controller task (which may
// touch flash), not in the RX callback. `value` is copied; safe to call from the
// promiscuous RX callback context. Non-blocking; drops if the queue is full.
void controller_notify_set_config(uint16_t key, const uint8_t *value, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif  // BULB_CONTROLLER_H
