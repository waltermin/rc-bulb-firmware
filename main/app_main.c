// app_main.c — startup ordering and module wiring.
//
// Ordering rule #1: light the LEDs before touching the radio. The custom PWM
// engine runs on the FRC1 hardware timer, which ticks from boot independent of
// Wi-Fi, so pwm_output_init() produces light immediately — no post-radio re-arm.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "config.h"
#include "pwm_output.h"
#include "bulb_config.h"
#include "sniffer.h"
#include "controller.h"
#include "dfu2.h"

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

    // 3. LEDs — GPIO/PWM configured, default color compiled and shown as a static
    //    level. Real PWM starts in step 7: the engine rides the Wi-Fi WDEV/TSF0
    //    timer (for glitch-free, Wi-Fi-priority edge timing), which needs the radio.
    pwm_output_init();

#ifdef DFU_ROLLBACK_GUARD
    // 4. If a freshly-flashed image keeps failing, revert before doing anything else.
    dfu2_rollback_check_on_boot();
#endif

    // 5. Identity.
    uint8_t my_id = bulb_config_get_u8(CFG_BULB_ID);

    // 6. Radio up in promiscuous mode.
    wifi_init_promiscuous();

    // 7. The WDEV/TSF0 timer the PWM engine rides only ticks now that the radio is
    //    started — begin real PWM output (the LED has shown a static default so far).
    pwm_output_start_after_radio();

    controller_start();
    sniffer_start();

    ESP_LOGI(TAG, "bulb %d running (fallback %u ms, channel %d)",
             my_id, bulb_config_get_u32(CFG_FALLBACK_MS),
             bulb_config_get_u8(CFG_WIFI_CHANNEL));

    // Report this image's build identity: the version string (from version.txt)
    // and the 8-byte DFU2 build id (sha256(version)[:8]) that the base station /
    // update server advertise, so a rollout is self-evident from the console.
    uint8_t bid[PROTO_DFU2_BUILD_ID_LEN];
    dfu2_own_build_id(bid);
    char bidhex[PROTO_DFU2_BUILD_ID_LEN * 2 + 1];
    for (int i = 0; i < PROTO_DFU2_BUILD_ID_LEN; i++) {
        snprintf(bidhex + i * 2, 3, "%02x", bid[i]);
    }
    ESP_LOGI(TAG, "build %s (id %s)", esp_ota_get_app_description()->version, bidhex);

#ifdef DFU_ROLLBACK_GUARD
    // 6. We reached a healthy running state; arm validation so this image is
    //    marked good after DFU_BOOT_VALIDATE_MS.
    dfu2_rollback_arm_validation();
#endif
}
