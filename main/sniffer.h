// sniffer.h — promiscuous-mode receive path. Parses beacons and forwards
// matches to the controller.

#ifndef BULB_SNIFFER_H
#define BULB_SNIFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Enable promiscuous mode (management frames only) on the configured Wi-Fi
// channel (CFG_WIFI_CHANNEL) and register the RX callback. Requires WiFi to be
// initialized and started in STA mode (but not connected), and the config store
// loaded. The bulb id we react to is read live from the config store on every
// received frame, so a SetConfig that changes it takes effect immediately.
void sniffer_start(void);

// Re-tune the promiscuous receiver to a new 802.11 channel at runtime. Called
// when a SetConfig changes CFG_WIFI_CHANNEL: unlike the id (a per-frame cache
// read), the channel is a radio setting that only changes when re-applied here.
// NOTE: if the base station is not broadcasting on `channel`, the bulb stops
// hearing all beacons — including further commands — until reboot or re-tune.
void sniffer_apply_channel(uint8_t channel);

// Disable promiscuous mode (used when transitioning into DFU).
void sniffer_stop(void);

// Forget the last Lemtone entry applied, so the next one is passed to the controller
// even if it is a repeat. Called by the controller when it falls back to the default
// color. Safe from any task (sets a flag the RX callback consumes).
void sniffer_forget_lemtone(void);

// Counters for the LemtoneUpdate path (diagnostics / self-test).
typedef struct {
    uint32_t posted;    // new values handed to the controller
    uint32_t repeats;   // unchanged repeats dropped (keep-alive sent instead)
    uint32_t dropped;   // new values lost to a full controller queue
} sniffer_lt_stats_t;
void sniffer_get_lt_stats(sniffer_lt_stats_t *out);

// Feed a whole 802.11 frame through the same path as a received one (parse,
// duplicate suppression, controller hand-off). For the on-device self-test; not
// synchronised with the RX callback, so only use it when no real traffic for this
// bulb is expected.
void sniffer_inject(const uint8_t *frame, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif  // BULB_SNIFFER_H
