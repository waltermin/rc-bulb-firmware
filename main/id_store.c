// id_store.c — bulb id persisted in NVS, outside the OTA app slots so it
// survives firmware updates.

#include "id_store.h"

#include "nvs.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "id_store";

#define ID_NVS_NAMESPACE "bulb"
#define ID_NVS_KEY "bulb_id"

uint8_t id_store_load(void) {
    nvs_handle handle;
    esp_err_t err = nvs_open(ID_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed (%d); using default id %d", err, BULB_DEFAULT_ID);
        return BULB_DEFAULT_ID;
    }

    uint8_t id = BULB_DEFAULT_ID;
    err = nvs_get_u8(handle, ID_NVS_KEY, &id);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no provisioned bulb_id (%d); using default id %d", err, BULB_DEFAULT_ID);
        return BULB_DEFAULT_ID;
    }

    ESP_LOGI(TAG, "bulb id = %d", id);
    return id;
}
