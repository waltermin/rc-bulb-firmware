// controller.c — single task that serializes all light-state changes.

#include "controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "config.h"
#include "pwm_output.h"
#include "dfu.h"

static const char *TAG = "controller";

typedef enum {
    MSG_APPLY = 0,
    MSG_DFU = 1,
} ctrl_msg_type_t;

typedef struct {
    uint8_t type;
    uint8_t r, g, b, ww, cw;
} ctrl_msg_t;

static QueueHandle_t s_queue;
static uint8_t s_my_id;

static void controller_task(void *arg) {
    (void)arg;
    TickType_t last_seen = 0;
    bool at_default = true;  // pwm_output_init already applied the default
    const TickType_t timeout_ticks = pdMS_TO_TICKS(FALLBACK_TIMEOUT_MS);

    for (;;) {
        ctrl_msg_t msg;
        // Wake at least once per second so the fallback timer stays responsive.
        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (msg.type == MSG_DFU) {
                ESP_LOGI(TAG, "DFU requested for id %d; entering DFU mode", s_my_id);
                // Hand off to a dedicated DFU task (large stack) which takes over
                // wifi, receives an image, and reboots. The controller is no
                // longer needed once DFU begins, so it stands down.
                dfu_start(s_my_id);
                vTaskDelete(NULL);
            }
            // MSG_APPLY — wire values are u8 (0..255); normalize to [0,1].
            pwm_output_set(msg.r  / 255.0f,
                           msg.g  / 255.0f,
                           msg.b  / 255.0f,
                           msg.ww / 255.0f,
                           msg.cw / 255.0f);
            last_seen = xTaskGetTickCount();
            at_default = false;
        }

        // Fallback check (runs on every wake, message or timeout).
        if (!at_default) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_seen) >= timeout_ticks) {
                ESP_LOGI(TAG, "no update in %d ms; reverting to default color", FALLBACK_TIMEOUT_MS);
                pwm_output_set_default();
                at_default = true;
            }
        }
    }
}

void controller_start(uint8_t my_id) {
    s_my_id = my_id;
    s_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    configASSERT(s_queue != NULL);
    xTaskCreate(controller_task, "controller", 4096, NULL, 5, NULL);
}

void controller_notify_entry(uint8_t r, uint8_t g, uint8_t b, uint8_t ww, uint8_t cw) {
    if (s_queue == NULL) {
        return;
    }
    ctrl_msg_t msg = {.type = MSG_APPLY, .r = r, .g = g, .b = b, .ww = ww, .cw = cw};
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
