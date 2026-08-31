// controller.h — owns light state: applies incoming colors, runs the fallback
// timeout, and triggers DFU mode. Fed by the sniffer RX path.

#ifndef BULB_CONTROLLER_H
#define BULB_CONTROLLER_H

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
// drops if the queue is momentarily full.
void controller_notify_entry(float r, float g, float b, float ww, float cw);

// Post a RAW color addressed to us (0x06 RawLightUpdate): duty cycles in [0,1]
// applied WITHOUT the gamma/perceptual curve or max-power cap (so they can
// overdrive), each channel with its own PWM period as a log2(ticks) exponent
// (clamped to the firmware range by the PWM layer). Resets the fallback timer like
// a normal color. Safe from the RX callback; non-blocking; drops if the queue is
// full. Color and period args are both in the fixed order r, g, b, ww, cw.
void controller_notify_raw_entry(float r, float g, float b, float ww, float cw,
                                 uint8_t period_r, uint8_t period_g, uint8_t period_b,
                                 uint8_t period_ww, uint8_t period_cw);

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
