// controller.c — single task that serializes all light-state changes.

#include "controller.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "config.h"
#include "bulb_config.h"
#include "protocol.h"
#include "pwm_output.h"
#include "sniffer.h"
#include "dfu.h"
#include "esp_system.h"  // esp_restart

static const char *TAG = "controller";

typedef enum {
    MSG_APPLY = 0,
    MSG_DFU = 1,
    MSG_SET_CONFIG = 2,
    MSG_REBOOT = 3,
} ctrl_msg_type_t;

typedef struct {
    uint8_t type;
    union {
        struct {
            float r, g, b, ww, cw;  // normalized [0,1] duty cycles (MSG_APPLY)
        } color;
        struct {
            uint16_t key;
            uint8_t  len;
            uint8_t  value[PROTO_CONFIG_VALUE_MAX];
        } cfg;                      // MSG_SET_CONFIG
    } u;
} ctrl_msg_t;

static QueueHandle_t s_queue;

// True if a SetConfig wire tag names one of the default/fallback color channels.
// A write to any of these changes the color the bulb shows while idle, so it is
// re-applied immediately when the bulb is currently at default.
static inline bool is_default_color_tag(uint16_t tag) {
    switch (tag) {
        case CFG_TAG_DEFAULT_R:
        case CFG_TAG_DEFAULT_G:
        case CFG_TAG_DEFAULT_B:
        case CFG_TAG_DEFAULT_WW:
        case CFG_TAG_DEFAULT_CW:
            return true;
        default:
            return false;
    }
}

static void controller_task(void *arg) {
    (void)arg;
    TickType_t last_seen = 0;
    bool at_default = true;  // pwm_output_init already applied the default

    for (;;) {
        ctrl_msg_t msg;
        // Wake at least once per second so the fallback timer stays responsive.
        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (msg.type == MSG_DFU) {
                // Read the id live so DFU uses the current CFG_BULB_ID even if a
                // SetConfig changed it since boot.
                const uint8_t my_id = bulb_config_get_u8(CFG_BULB_ID);
                ESP_LOGI(TAG, "DFU requested for id %d; entering DFU mode", my_id);
                // Hand off to a dedicated DFU task (large stack) which takes over
                // wifi, receives an image, and reboots. The controller is no
                // longer needed once DFU begins, so it stands down.
                dfu_start(my_id);
                vTaskDelete(NULL);
            } else if (msg.type == MSG_REBOOT) {
                ESP_LOGI(TAG, "reboot requested; restarting");
                esp_restart();  // does not return
            } else if (msg.type == MSG_SET_CONFIG) {
                // Config writes touch flash, so they happen here rather than in
                // the RX callback. Does not affect the fallback timer.
                esp_err_t err = bulb_config_set_raw(msg.u.cfg.key, msg.u.cfg.value, msg.u.cfg.len);
                ESP_LOGI(TAG, "set config key 0x%04x (%u bytes) (err=%d)",
                         msg.u.cfg.key, msg.u.cfg.len, err);
                // If a default-color channel changed and we're currently resting
                // at the default, repaint now so the new default takes effect
                // instantly instead of waiting for the next fallback timeout. If a
                // live color is showing (at_default == false) we leave it be: the
                // new default already takes over at the next fallback, and forcing
                // it here would only flash until the next beacon overwrites it.
                if (err == ESP_OK && at_default && is_default_color_tag(msg.u.cfg.key)) {
                    pwm_output_set_default();
                }
                // The Wi-Fi channel is a radio setting, not a per-frame cache
                // read, so a change only takes effect when we re-tune the
                // receiver here. (Id and fallback timeout are read live and need
                // no such kick.)
                if (err == ESP_OK && msg.u.cfg.key == CFG_TAG_WIFI_CHANNEL) {
                    sniffer_apply_channel(bulb_config_get_u8(CFG_WIFI_CHANNEL));
                }
            } else {
                // MSG_APPLY — colors arrive already normalized to [0,1] by the parser.
                pwm_output_set(msg.u.color.r, msg.u.color.g, msg.u.color.b,
                               msg.u.color.ww, msg.u.color.cw);
                last_seen = xTaskGetTickCount();
                at_default = false;
            }
        }

        // Fallback check (runs on every wake, message or timeout). The timeout is
        // read live from the config store (this task is also its only writer, so
        // no locking is needed) — a SetConfig that changes CFG_FALLBACK_MS is
        // honored from the next check on, no reboot required.
        if (!at_default) {
            const uint32_t fallback_ms = bulb_config_get_u32(CFG_FALLBACK_MS);
            TickType_t now = xTaskGetTickCount();
            if ((now - last_seen) >= pdMS_TO_TICKS(fallback_ms)) {
                ESP_LOGI(TAG, "no update in %u ms; reverting to default color", fallback_ms);
                pwm_output_set_default();
                at_default = true;
            }
        }
    }
}

void controller_start(void) {
    s_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    configASSERT(s_queue != NULL);
    xTaskCreate(controller_task, "controller", 4096, NULL, 5, NULL);
}

void controller_notify_entry(float r, float g, float b, float ww, float cw) {
    if (s_queue == NULL) {
        return;
    }
    ctrl_msg_t msg = {.type = MSG_APPLY,
                      .u.color = {.r = r, .g = g, .b = b, .ww = ww, .cw = cw}};
    // Drop if full: a newer beacon will arrive shortly anyway.
    xQueueSend(s_queue, &msg, 0);
}

void controller_notify_dfu(void) {
    if (s_queue == NULL) {
        return;
    }
    ctrl_msg_t msg = {.type = MSG_DFU};
    xQueueSend(s_queue, &msg, 0);
}

void controller_notify_reboot(void) {
    if (s_queue == NULL) {
        return;
    }
    ctrl_msg_t msg = {.type = MSG_REBOOT};
    xQueueSend(s_queue, &msg, 0);
}

void controller_notify_set_config(uint16_t key, const uint8_t *value, uint8_t len) {
    if (s_queue == NULL) {
        return;
    }
    if (len > PROTO_CONFIG_VALUE_MAX) {
        return;  // parser already caps this, but never overrun the buffer
    }
    ctrl_msg_t msg = {.type = MSG_SET_CONFIG};
    msg.u.cfg.key = key;
    msg.u.cfg.len = len;
    if (len > 0 && value != NULL) {
        memcpy(msg.u.cfg.value, value, len);
    }
    xQueueSend(s_queue, &msg, 0);
}
