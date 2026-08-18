// id_store.h — read the bulb's u8 id from NVS (provisioned per device).

#ifndef BULB_ID_STORE_H
#define BULB_ID_STORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Read the provisioned id from NVS namespace "bulb", key "bulb_id". Returns the
// stored value, or BULB_DEFAULT_ID if the key is missing / NVS unavailable.
// Requires nvs_flash_init() to have been called first.
uint8_t id_store_load(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_ID_STORE_H
