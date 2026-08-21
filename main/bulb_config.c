// bulb_config.c — runtime config store backed by NVS, cached in RAM.
//
// One descriptor row per key describes where it lives in NVS (namespace + key),
// its type, and its compile-time default (from config.h). bulb_config_init()
// loads every key into s_cache once; getters read the cache; setters write the
// cache and persist to NVS.

#include "bulb_config.h"

#include <stdbool.h>
#include <string.h>

#include "nvs.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "config";

// NVS namespaces. "bulb" is the legacy namespace holding the provisioned id
// (written by tools/provision_id.py); "cfg" holds everything else. The rollback
// guard's "dfu" namespace is owned elsewhere and untouched here.
#define NS_BULB "bulb"
#define NS_CFG "cfg"

// Cache buffer for string values: fits a 63-char WPA2 PSK + NUL (and any SSID).
#define CFG_STR_MAX 64

typedef enum { CFG_T_U8, CFG_T_U32, CFG_T_FLOAT, CFG_T_STR } cfg_type_t;

typedef struct {
    const char *ns;   // NVS namespace
    const char *key;  // NVS key, <= 15 chars (NVS limit)
    cfg_type_t  type;
    union {
        uint8_t     u8;
        uint32_t    u32;
        float       f;
        const char *str;
    } def;
} cfg_desc_t;

// Adding a key: add a cfg_key_t enum value (header) and a row here. Defaults are
// the config.h macros — the single source of truth for absent-key values.
static const cfg_desc_t CFG_DESC[CFG_KEY_COUNT] = {
    [CFG_BULB_ID]      = {NS_BULB, "bulb_id",     CFG_T_U8,    .def.u8  = BULB_DEFAULT_ID},
    [CFG_DEFAULT_R]    = {NS_CFG,  "def_r",       CFG_T_FLOAT, .def.f   = DEFAULT_R},
    [CFG_DEFAULT_G]    = {NS_CFG,  "def_g",       CFG_T_FLOAT, .def.f   = DEFAULT_G},
    [CFG_DEFAULT_B]    = {NS_CFG,  "def_b",       CFG_T_FLOAT, .def.f   = DEFAULT_B},
    [CFG_DEFAULT_WW]   = {NS_CFG,  "def_ww",      CFG_T_FLOAT, .def.f   = DEFAULT_WW},
    [CFG_DEFAULT_CW]   = {NS_CFG,  "def_cw",      CFG_T_FLOAT, .def.f   = DEFAULT_CW},
    [CFG_FALLBACK_MS]  = {NS_CFG,  "fallback_ms", CFG_T_U32,   .def.u32 = FALLBACK_TIMEOUT_MS},
    [CFG_WIFI_CHANNEL] = {NS_CFG,  "wifi_chan",   CFG_T_U8,    .def.u8  = WIFI_CHANNEL},
    [CFG_DUTY_CURVE]   = {NS_CFG,  "duty_curve",  CFG_T_U8,    .def.u8  = PWM_DUTY_CURVE},
    [CFG_GAMMA]        = {NS_CFG,  "gamma",       CFG_T_FLOAT, .def.f   = PWM_GAMMA},
    [CFG_DFU_SSID]     = {NS_CFG,  "dfu_ssid",    CFG_T_STR,   .def.str = DFU_AP_SSID},
    [CFG_DFU_PASS]     = {NS_CFG,  "dfu_pass",    CFG_T_STR,   .def.str = DFU_AP_PASS},
};

// RAM cache. Scalars sit in a 4-byte-aligned union (single-word loads on Xtensa
// are atomic, so a hot-path reader never tears against a setter). Strings are
// stored inline, always NUL-terminated.
typedef struct {
    union {
        uint8_t  u8;
        uint32_t u32;
        float    f;
    } scalar;
    char str[CFG_STR_MAX];
} cfg_entry_t;

static cfg_entry_t s_cache[CFG_KEY_COUNT];

// Copy a key's compile-time default into its cache entry.
static void load_default(cfg_key_t k) {
    const cfg_desc_t *d = &CFG_DESC[k];
    cfg_entry_t *e = &s_cache[k];
    switch (d->type) {
        case CFG_T_U8:    e->scalar.u8  = d->def.u8;  break;
        case CFG_T_U32:   e->scalar.u32 = d->def.u32; break;
        case CFG_T_FLOAT: e->scalar.f   = d->def.f;   break;
        case CFG_T_STR:
            strncpy(e->str, d->def.str, CFG_STR_MAX - 1);
            e->str[CFG_STR_MAX - 1] = '\0';
            break;
    }
}

// Load a single key from an open NVS handle into its cache entry. Returns ESP_OK
// only if the stored value was read intact; any error leaves the entry untouched
// (caller falls back to the default).
static esp_err_t load_from_nvs(nvs_handle handle, cfg_key_t k) {
    const cfg_desc_t *d = &CFG_DESC[k];
    cfg_entry_t *e = &s_cache[k];
    switch (d->type) {
        case CFG_T_U8:
            return nvs_get_u8(handle, d->key, &e->scalar.u8);
        case CFG_T_U32:
            return nvs_get_u32(handle, d->key, &e->scalar.u32);
        case CFG_T_FLOAT: {
            // NVS has no float type; a float is stored as a 4-byte blob.
            size_t len = sizeof(float);
            esp_err_t err = nvs_get_blob(handle, d->key, &e->scalar.f, &len);
            if (err == ESP_OK && len != sizeof(float)) {
                return ESP_ERR_NVS_INVALID_LENGTH;
            }
            return err;
        }
        case CFG_T_STR: {
            size_t len = 0;
            esp_err_t err = nvs_get_str(handle, d->key, NULL, &len);
            if (err != ESP_OK) {
                return err;
            }
            if (len > CFG_STR_MAX) {  // too long for the cache; reject -> default
                return ESP_ERR_NVS_INVALID_LENGTH;
            }
            return nvs_get_str(handle, d->key, e->str, &len);
        }
    }
    return ESP_FAIL;
}

void bulb_config_init(void) {
    // Open each namespace once (read-only). A namespace that was never written
    // (e.g. "cfg" on an already-provisioned device) fails to open; all its keys
    // then take their defaults.
    nvs_handle h_bulb = 0, h_cfg = 0;
    bool have_bulb = (nvs_open(NS_BULB, NVS_READONLY, &h_bulb) == ESP_OK);
    bool have_cfg  = (nvs_open(NS_CFG,  NVS_READONLY, &h_cfg) == ESP_OK);

    for (cfg_key_t k = 0; k < CFG_KEY_COUNT; k++) {
        const cfg_desc_t *d = &CFG_DESC[k];
        nvs_handle h = 0;
        bool have = false;
        if (strcmp(d->ns, NS_BULB) == 0) {
            h = h_bulb;
            have = have_bulb;
        } else {
            h = h_cfg;
            have = have_cfg;
        }
        if (!have || load_from_nvs(h, k) != ESP_OK) {
            load_default(k);
        }
    }

    if (have_bulb) nvs_close(h_bulb);
    if (have_cfg)  nvs_close(h_cfg);

    ESP_LOGI(TAG, "config loaded: id=%u chan=%u curve=%u fallback=%ums",
             s_cache[CFG_BULB_ID].scalar.u8, s_cache[CFG_WIFI_CHANNEL].scalar.u8,
             s_cache[CFG_DUTY_CURVE].scalar.u8, s_cache[CFG_FALLBACK_MS].scalar.u32);
}

uint8_t bulb_config_get_u8(cfg_key_t k) {
    return s_cache[k].scalar.u8;
}

uint32_t bulb_config_get_u32(cfg_key_t k) {
    return s_cache[k].scalar.u32;
}

float bulb_config_get_float(cfg_key_t k) {
    return s_cache[k].scalar.f;
}

const char *bulb_config_get_str(cfg_key_t k) {
    return s_cache[k].str;
}

// Persist a cache entry to NVS (open read-write, set by type, commit, close).
static esp_err_t persist(cfg_key_t k) {
    const cfg_desc_t *d = &CFG_DESC[k];
    cfg_entry_t *e = &s_cache[k];
    nvs_handle h;
    esp_err_t err = nvs_open(d->ns, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    switch (d->type) {
        case CFG_T_U8:    err = nvs_set_u8(h, d->key, e->scalar.u8);   break;
        case CFG_T_U32:   err = nvs_set_u32(h, d->key, e->scalar.u32); break;
        case CFG_T_FLOAT: err = nvs_set_blob(h, d->key, &e->scalar.f, sizeof(float)); break;
        case CFG_T_STR:   err = nvs_set_str(h, d->key, e->str);        break;
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t bulb_config_set_u8(cfg_key_t k, uint8_t v) {
    s_cache[k].scalar.u8 = v;
    return persist(k);
}

esp_err_t bulb_config_set_u32(cfg_key_t k, uint32_t v) {
    s_cache[k].scalar.u32 = v;
    return persist(k);
}

esp_err_t bulb_config_set_float(cfg_key_t k, float v) {
    s_cache[k].scalar.f = v;
    return persist(k);
}

esp_err_t bulb_config_set_str(cfg_key_t k, const char *v) {
    strncpy(s_cache[k].str, v, CFG_STR_MAX - 1);
    s_cache[k].str[CFG_STR_MAX - 1] = '\0';
    return persist(k);
}
