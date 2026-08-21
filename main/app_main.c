// app_main.c — startup ordering and module wiring.
//
// Ordering rule #1: configure the LEDs before touching the radio. The SDK PWM
// driver clocks its ISR off the Wi-Fi (WDEV/TSF0) timer, which does not tick
// until esp_wifi_start(), so actual light output is (re)armed immediately after
// the radio comes up via pwm_output_start_after_radio() — as early as the
// hardware allows.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "config.h"
#include "pwm_output.h"
#include "bulb_config.h"
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
    // 1. Persistent storage (config + id + rollback state) must come first: the
    //    PWM setup below now reads its default color, curve, and gamma from the
    //    config store.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. Load the config cache (NVS-or-default) before any consumer reads it.
    bulb_config_init();

    // 3. LEDs — GPIO/PWM configured and default color primed. No radio yet, so
    //    this does not produce light on its own (see step 7); its only effect is
    //    GPIO setup, so running it a few ms after NVS init is harmless.
    pwm_output_init();

#ifdef DFU_ROLLBACK_GUARD
    // 4. If a freshly-flashed image keeps failing, revert before doing anything else.
    dfu_rollback_check_on_boot();
#endif

    // 5. Identity.
    uint8_t my_id = bulb_config_get_u8(CFG_BULB_ID);

    // 6. Radio up in promiscuous mode.
    wifi_init_promiscuous();

    // 7. The WDEV timer the PWM driver rides on is only clocked now that the
    //    radio is started — actually begin LED output at the default color.
    pwm_output_start_after_radio();

    controller_start(my_id);
    sniffer_start(my_id);

    ESP_LOGI(TAG, "bulb %d running (fallback %u ms, channel %d)",
             my_id, bulb_config_get_u32(CFG_FALLBACK_MS),
             bulb_config_get_u8(CFG_WIFI_CHANNEL));

#ifdef DFU_ROLLBACK_GUARD
    // 6. We reached a healthy running state; arm validation so this image is
    //    marked good after DFU_BOOT_VALIDATE_MS.
    dfu_rollback_arm_validation();
#endif
}
