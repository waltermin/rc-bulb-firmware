// controller.h — owns light state: applies incoming colors, runs the fallback
// timeout, and triggers DFU mode. Fed by the sniffer RX path.

#ifndef BULB_CONTROLLER_H
#define BULB_CONTROLLER_H

#include <stdint.h>

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

// Post a DFU request (safe to call from the promiscuous RX callback context).
void controller_notify_dfu(void);

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
