// app_main.c — startup ordering and module wiring.
//
// Ordering rule #1: drive the LEDs before touching the radio, so the bulb lights
// up within tens of milliseconds of power-on.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "config.h"
#include "pwm_output.h"
#include "id_store.h"
#include "sniffer.h"
#include "controller.h"
#include "dfu.h"

static const char *TAG = "app";

static void wifi_init_promiscuous(void) {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // Keep wifi config in RAM; identity lives in our own NVS key, and we never
    // want stale saved STA credentials interfering with promiscuous operation.
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Started in STA mode but deliberately NOT connected: promiscuous only.
}

void app_main(void) {
    // 1. LEDs first — no radio yet.
    pwm_output_init();

    // 2. Persistent storage (id + rollback state).
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

#ifdef DFU_ROLLBACK_GUARD
    // 3. If a freshly-flashed image keeps failing, revert before doing anything else.
    dfu_rollback_check_on_boot();
#endif

    // 4. Identity.
    uint8_t my_id = id_store_load();

    // 5. Radio up in promiscuous mode.
    wifi_init_promiscuous();
    controller_start(my_id);
    sniffer_start(my_id);

    ESP_LOGI(TAG, "bulb %d running (fallback %d ms, channel %d)",
             my_id, FALLBACK_TIMEOUT_MS, WIFI_CHANNEL);

#ifdef DFU_ROLLBACK_GUARD
    // 6. We reached a healthy running state; arm validation so this image is
    //    marked good after DFU_BOOT_VALIDATE_MS.
    dfu_rollback_arm_validation();
#endif
}
