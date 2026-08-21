// bulb_config.h — runtime, NVS-backed configuration store.
//
// Every operator-tunable value lives in NVS and is read at runtime through the
// typed getters below. config.h holds the compile-time DEFAULT for each value,
// used only when the key is absent from flash. All values are loaded into a RAM
// cache once at startup (bulb_config_init), so reads are fast enough for the
// PWM hot path.
//
// Adding a new key: add an entry to cfg_key_t here and a matching row to the
// descriptor table in bulb_config.c (nvs key, type, default). A key's value
// type is assumed never to change once shipped.

#ifndef BULB_CONFIG_MODULE_H
#define BULB_CONFIG_MODULE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Stable identity for each tunable. The numeric id is a compile-time index into
// the RAM cache and descriptor table; it never touches flash. The on-flash
// identity is the NVS string key in the descriptor.
typedef enum {
    CFG_BULB_ID = 0,   // u8    — ns "bulb" key "bulb_id" (legacy location, unchanged)
    CFG_DEFAULT_R,     // float — default/fallback color, normalized [0,1]
    CFG_DEFAULT_G,     // float
    CFG_DEFAULT_B,     // float
    CFG_DEFAULT_WW,    // float
    CFG_DEFAULT_CW,    // float
    CFG_FALLBACK_MS,   // u32   — revert-to-default timeout, ms
    CFG_WIFI_CHANNEL,  // u8    — 802.11 channel to sniff
    CFG_DUTY_CURVE,    // u8    — PWM_CURVE_GAMMA / PWM_CURVE_PERCEPTUAL
    CFG_GAMMA,         // float — gamma exponent when curve == PWM_CURVE_GAMMA
    CFG_DFU_SSID,      // str   — AP to join for DFU
    CFG_DFU_PASS,      // str   — AP password
    CFG_KEY_COUNT,
} cfg_key_t;

// Populate the whole RAM cache from NVS-or-default. Call exactly once, after
// nvs_flash_init() and before any consumer reads config. Never fails: any
// per-key NVS error (including a missing key or a never-provisioned namespace)
// falls back to that key's config.h default.
void bulb_config_init(void);

// Typed cache reads. Using the wrong accessor for a key's type is UB by design
// (values are raw bytes; callers must know the type).
uint8_t     bulb_config_get_u8(cfg_key_t k);
uint32_t    bulb_config_get_u32(cfg_key_t k);
float       bulb_config_get_float(cfg_key_t k);
const char *bulb_config_get_str(cfg_key_t k);  // always NUL-terminated

// Write-through setters: update the RAM cache and persist to NVS (set + commit).
// Provided so the store is writable (provisioning / future use); no runtime
// caller today. Return ESP_OK or the underlying NVS error.
esp_err_t bulb_config_set_u8(cfg_key_t k, uint8_t v);
esp_err_t bulb_config_set_u32(cfg_key_t k, uint32_t v);
esp_err_t bulb_config_set_float(cfg_key_t k, float v);
esp_err_t bulb_config_set_str(cfg_key_t k, const char *v);

#ifdef __cplusplus
}
#endif

#endif  // BULB_CONFIG_MODULE_H
