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

// Wire tags: the 16-bit key identifiers used by the SetConfig BulbCommand
// (spec 0x04 / cmd 0x01). Stable across firmware versions; a key's value type
// never changes once assigned. Grouped loosely by area. These map to cfg_key_t
// via the descriptor table in bulb_config.c and are the identity callers use
// with bulb_config_set_raw().
#define CFG_TAG_BULB_ID      0x0001u
#define CFG_TAG_DEFAULT_R    0x0010u
#define CFG_TAG_DEFAULT_G    0x0011u
#define CFG_TAG_DEFAULT_B    0x0012u
#define CFG_TAG_DEFAULT_WW   0x0013u
#define CFG_TAG_DEFAULT_CW   0x0014u
#define CFG_TAG_FALLBACK_MS  0x0020u
#define CFG_TAG_WIFI_CHANNEL 0x0021u
#define CFG_TAG_DUTY_CURVE   0x0030u
#define CFG_TAG_GAMMA        0x0031u
#define CFG_TAG_DFU_SSID     0x0040u
#define CFG_TAG_DFU_PASS     0x0041u

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

// Set a key identified by its 16-bit wire tag (CFG_TAG_*) from raw bytes, as
// delivered by a SetConfig BulbCommand. The byte length must match the key's
// type (1 for u8, 4 for u32/float, up to 63 for strings); a mismatch or unknown
// tag is rejected. Updates the RAM cache and persists to NVS. Returns ESP_OK,
// ESP_ERR_NVS_NOT_FOUND (unknown tag), or ESP_ERR_INVALID_SIZE (bad length).
esp_err_t bulb_config_set_raw(uint16_t tag, const void *value, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif  // BULB_CONFIG_MODULE_H
