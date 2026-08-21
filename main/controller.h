// controller.h — owns light state: applies incoming colors, runs the fallback
// timeout, and triggers DFU mode. Fed by the sniffer RX path.

#ifndef BULB_CONTROLLER_H
#define BULB_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Create the controller queue + task. my_id is used only for logging here; the
// sniffer does the id matching.
void controller_start(uint8_t my_id);

// Post a new color addressed to us, as normalized duty cycles in [0.0, 1.0]
// (the protocol parser has already scaled/clamped both packet types into this
// range). Safe to call from the promiscuous RX callback context. Non-blocking;
// drops if the queue is momentarily full.
void controller_notify_entry(float r, float g, float b, float ww, float cw);

// Post a DFU request (safe to call from the promiscuous RX callback context).
void controller_notify_dfu(void);

// Post a config write (from a SetConfig BulbCommand): `key` is the 16-bit wire
// tag, `value`/`len` the raw bytes. Applied on the controller task (which may
// touch flash), not in the RX callback. `value` is copied; safe to call from the
// promiscuous RX callback context. Non-blocking; drops if the queue is full.
void controller_notify_set_config(uint16_t key, const uint8_t *value, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif  // BULB_CONTROLLER_H
