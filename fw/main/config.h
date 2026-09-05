// config.h — tunables for the bare-SDK rc-switch firmware (Tuya T1-2S-NL / BK7238).
//
// Ground-up firmware built directly on beken_freertos_sdk (no OpenBeken OS).
// Wire-format constants live in protocol.h; this is device/behavior config.

#ifndef RCSW_CONFIG_H
#define RCSW_CONFIG_H

// ---- identity (shared bulb/relay id space) ----
#define RCSW_MY_ID            5

// ---- hardware pins (BK7238 GPIO index == pin number) ----
#define RCSW_PIN_RELAY        9    // P9, relay drive, active-high
#define RCSW_PIN_LED          6    // P6, status LED
#define RCSW_PIN_BUTTON       8    // P8, unused by design (spec: no function)

// ---- defaults + fallback (spec: applied at boot and on packet silence) ----
#define RCSW_DEFAULT_RELAY    0
#define RCSW_DEFAULT_LED      0
#define RCSW_FALLBACK_MS      15000

// ---- radio ----
#define RCSW_WIFI_CHANNEL     11

#endif  // RCSW_CONFIG_H
