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

// Post a new color addressed to us (safe to call from the promiscuous RX
// callback context). Non-blocking; drops if the queue is momentarily full.
void controller_notify_entry(uint8_t r, uint8_t g, uint8_t b, uint8_t ww, uint8_t cw);

// Post a DFU request (safe to call from the promiscuous RX callback context).
void controller_notify_dfu(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_CONTROLLER_H
