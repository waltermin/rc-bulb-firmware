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
// loaded. my_id selects which BulbEntry we react to.
void sniffer_start(uint8_t my_id);

// Disable promiscuous mode (used when transitioning into DFU).
void sniffer_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_SNIFFER_H
