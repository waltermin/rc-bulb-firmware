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

#ifdef __cplusplus
}
#endif

#endif  // BULB_SNIFFER_H
