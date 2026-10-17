// settings.h — persisted, OTA-updatable device configuration.
//
// The values in config.h are the COMPILE-TIME DEFAULTS. At boot we load a
// persisted copy from a dedicated flash sector; if that sector is blank or
// fails validation (magic + version + CRC32) we fall back to those defaults.
//
// A 0x04 BulbCommand / SetConfig (key,len,value) mutates a field in place and,
// when the value actually changes, is written back to flash. This is the relay
// device's runtime-config path — no rebuild/DFU required to change id, channel,
// power-on defaults, or the fallback window.
//
// Wire-format note: SetConfig keys/values are part of the ecosystem ABI (a base
// station broadcasts them). Keys are u16; values are little-endian and length-
// checked. Keep these stable and mirror them into spec.txt.

#ifndef RCSW_SETTINGS_H
#define RCSW_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t  my_id;         // shared bulb/relay id space
    uint8_t  wifi_channel;  // 1..14
    uint8_t  default_relay; // 0/1 — power-on + fallback relay state
    uint8_t  default_led;   // 0/1 — power-on + fallback LED state
    uint32_t fallback_ms;   // per-packet-type silence window
} rcsw_settings_t;

// SetConfig wire keys (u16). ONE FLAT KEY SPACE shared with the bulbs
// (bulb_config.h CFG_TAG_*): a key number means the same thing on every device
// type that implements it, and unknown keys are ignored by everyone. Concepts
// the bulbs already define are REUSED (device id, fallback, channel — same
// types); switch-specific settings live at 0x005X. Value encodings per key.
#define RCSW_CFG_KEY_MY_ID          0x0001u  // u8  (bulbs: CFG_TAG_BULB_ID — shared "device id")
#define RCSW_CFG_KEY_FALLBACK_MS    0x0020u  // u32 little-endian, ms (bulbs: CFG_TAG_FALLBACK_MS)
#define RCSW_CFG_KEY_WIFI_CHANNEL   0x0021u  // u8  1..14 (bulbs: CFG_TAG_WIFI_CHANNEL)
#define RCSW_CFG_KEY_DEFAULT_RELAY  0x0050u  // u8  (0/1) — switch-specific
#define RCSW_CFG_KEY_DEFAULT_LED    0x0051u  // u8  (0/1) — switch-specific

// Fill *out from flash, or from the config.h defaults if unpersisted/invalid.
void rcsw_settings_load(rcsw_settings_t *out);

// Apply one SetConfig (key, value[len]) to *s in place. Returns true iff a field
// actually changed (caller then persists). Unknown keys, bad lengths, and
// out-of-range values are ignored (returns false) — a malformed command is a
// no-op, never a brick.
bool rcsw_settings_apply(rcsw_settings_t *s, uint16_t key,
                         const uint8_t *val, uint8_t len);

// Persist *s to its flash sector (erase + write + read-back verify).
// Returns true on a verified write.
bool rcsw_settings_save(const rcsw_settings_t *s);

#endif  // RCSW_SETTINGS_H
