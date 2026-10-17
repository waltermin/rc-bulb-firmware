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
#include "dfu2.h"
#include "esp_system.h"  // esp_restart
#include "esp_timer.h"   // esp_timer_get_time (Lemtone timing log)
#include "lemtone.h"

static const char *TAG = "controller";

typedef enum {
    MSG_APPLY = 0,
    MSG_DFU2 = 1,
    MSG_SET_CONFIG = 2,
    MSG_REBOOT = 3,
    MSG_APPLY_RAW = 4,
    MSG_APPLY_LEMTONE = 5,
    MSG_KEEPALIVE = 6,
} ctrl_msg_type_t;

typedef struct {
    uint8_t type;
    union {
        struct {
            float r, g, b, ww, cw;  // normalized [0,1] duty cycles (MSG_APPLY)
        } color;
        struct {
            float   r, g, b, ww, cw;         // raw duties, clamp-only (MSG_APPLY_RAW)
            uint8_t pr, pg, pb, pww, pcw;    // per-channel PWM period, log2(ticks)
        } raw;                      // MSG_APPLY_RAW
        struct {
            float a, b, lm, f;      // decoded Lemtone value (MSG_APPLY_LEMTONE)
        } lemtone;
        struct {
            uint16_t key;
            uint8_t  len;
            uint8_t  value[PROTO_CONFIG_VALUE_MAX];
        } cfg;                      // MSG_SET_CONFIG
        dfu2_params_t dfu2;         // MSG_DFU2
    } u;
} ctrl_msg_t;

static QueueHandle_t s_queue;

// Lemtone solver context: built once from the compiled-in model, read-only after.
static lemtone_ctx_t s_lemtone;
static controller_lemtone_stats_t s_lt_stats = {.min_stack_free = UINT32_MAX};

void controller_get_lemtone_stats(controller_lemtone_stats_t *out) {
    *out = s_lt_stats;
}

// Solve a Lemtone value and drive the PWM with the result. The solver output is
// already capped at the model's duty_max and quantized to whole ticks per period, so
// it goes through the raw path (no curve, no extra cap); pwm_compile turns each
// cmd_duty back into exactly the solver's tick count.
static void apply_lemtone(float a, float b, float lm, float f) {
    const int64_t t0 = esp_timer_get_time();
    lemtone_solution_t sol;
    lemtone_pwm_t pwm;
    lemtone_compute(&s_lemtone, &LEMTONE_PERIOD_DEFAULT, a, b, lm, f, &sol, &pwm);
    const int64_t t1 = esp_timer_get_time();
    pwm_output_set_raw(pwm.cmd_duty, pwm.period_log2);
    const int64_t t2 = esp_timer_get_time();

    controller_lemtone_stats_t *st = &s_lt_stats;
    st->applies++;
    st->last_mode = sol.mode;
    st->last_evals = sol.evals;
    st->last_solve_us = (uint32_t)(t1 - t0);
    st->last_pwm_us = (uint32_t)(t2 - t1);
    if (st->last_solve_us > st->max_solve_us) st->max_solve_us = st->last_solve_us;
    if (st->last_pwm_us > st->max_pwm_us) st->max_pwm_us = st->last_pwm_us;
    const uint32_t free_b = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (free_b < st->min_stack_free) st->min_stack_free = free_b;
#if LEMTONE_TIMING_LOG
    // integers only: mlm = millilumens, desat in 0.1% (no float printf on this libc)
    ESP_LOGI(TAG, "lemtone mode %u evals %u solve %d us pwm %d us | %u mlm desat %u/1000 | "
             "ticks %u %u %u %u %u @2^%u %u %u %u %u | stack free %u B",
             sol.mode, sol.evals, (int)(t1 - t0), (int)(t2 - t1),
             (unsigned)(sol.lm * 1000.0f), (unsigned)(sol.desat * 1000.0f),
             pwm.ticks[0], pwm.ticks[1], pwm.ticks[2], pwm.ticks[3], pwm.ticks[4],
             pwm.period_log2[0], pwm.period_log2[1], pwm.period_log2[2], pwm.period_log2[3],
             pwm.period_log2[4], (unsigned)uxTaskGetStackHighWaterMark(NULL));
#endif
}

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
            if (msg.type == MSG_DFU2) {
                // Read the id live so DFU2 uses the current CFG_BULB_ID even if a
                // SetConfig changed it since boot.
                const uint8_t my_id = bulb_config_get_u8(CFG_BULB_ID);
                ESP_LOGI(TAG, "DFU2 requested for id %d; entering DFU2 mode", my_id);
                // Hand off to a dedicated DFU2 task (large stack) which takes over
                // wifi, pulls an image, and reboots. The controller is no longer
                // needed once DFU2 begins, so it stands down.
                dfu2_start(my_id, &msg.u.dfu2);
                vTaskDelete(NULL);
            } else if (msg.type == MSG_REBOOT) {
                ESP_LOGI(TAG, "reboot requested; restarting");
                esp_restart();  // does not return
            } else if (msg.type == MSG_SET_CONFIG) {
                // Config writes touch flash, so they happen here rather than in
                // the RX callback. Does not affect the fallback timer.
                esp_err_t err = bulb_config_set_raw(msg.u.cfg.key, msg.u.cfg.value, msg.u.cfg.len);
                ESP_LOGI(TAG, "set config key 0x%04x (%u bytes) (err=%d)",
                         (unsigned)msg.u.cfg.key, (unsigned)msg.u.cfg.len, (int)err);
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
            } else if (msg.type == MSG_APPLY_LEMTONE) {
                apply_lemtone(msg.u.lemtone.a, msg.u.lemtone.b, msg.u.lemtone.lm, msg.u.lemtone.f);
                last_seen = xTaskGetTickCount();
                at_default = false;
            } else if (msg.type == MSG_KEEPALIVE) {
                // An unchanged repeat of what we show: only the fallback timer moves.
                // Once at the default, the sniffer has been told to re-apply its next
                // entry, so a stale keep-alive must not hold off nothing.
                if (!at_default) {
                    last_seen = xTaskGetTickCount();
                }
            } else if (msg.type == MSG_APPLY_RAW) {
                // MSG_APPLY_RAW — raw duties (clamp-only, no curve/cap) plus a
                // per-channel PWM period. Arrays are in COLOR_* order R,G,B,WW,CW.
                const float duties[5] = {msg.u.raw.r, msg.u.raw.g, msg.u.raw.b,
                                         msg.u.raw.ww, msg.u.raw.cw};
                const uint8_t periods[5] = {msg.u.raw.pr, msg.u.raw.pg, msg.u.raw.pb,
                                            msg.u.raw.pww, msg.u.raw.pcw};
                pwm_output_set_raw(duties, periods);
                last_seen = xTaskGetTickCount();
                at_default = false;
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
                ESP_LOGI(TAG, "no update in %u ms; reverting to default color",
                         (unsigned)fallback_ms);
                pwm_output_set_default();
                at_default = true;
                s_lt_stats.fallbacks++;
                // The sniffer drops Lemtone repeats; the next one must repaint.
                sniffer_forget_lemtone();
            }
        }
    }
}

void controller_start(void) {
    lemtone_init(&s_lemtone, &LEMTONE_MODEL_DEFAULT);
    ESP_LOGI(TAG, "lemtone model %s", s_lemtone.m->id);
    s_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    configASSERT(s_queue != NULL);
    xTaskCreate(controller_task, "controller", 4096, NULL, 5, NULL);
}

bool controller_notify_entry(float r, float g, float b, float ww, float cw) {
    if (s_queue == NULL) {
        return false;
    }
    ctrl_msg_t msg = {.type = MSG_APPLY,
                      .u.color = {.r = r, .g = g, .b = b, .ww = ww, .cw = cw}};
    // Drop if full: a newer beacon will arrive shortly anyway.
    return xQueueSend(s_queue, &msg, 0) == pdTRUE;
}

bool controller_notify_raw_entry(float r, float g, float b, float ww, float cw,
                                 uint8_t pr, uint8_t pg, uint8_t pb,
                                 uint8_t pww, uint8_t pcw) {
    if (s_queue == NULL) {
        return false;
    }
    ctrl_msg_t msg = {.type = MSG_APPLY_RAW,
                      .u.raw = {.r = r, .g = g, .b = b, .ww = ww, .cw = cw,
                                .pr = pr, .pg = pg, .pb = pb, .pww = pww, .pcw = pcw}};
    // Drop if full: a newer beacon will arrive shortly anyway.
    return xQueueSend(s_queue, &msg, 0) == pdTRUE;
}

bool controller_notify_lemtone(float a, float b, float lm, float fidelity) {
    if (s_queue == NULL) {
        return false;
    }
    ctrl_msg_t msg = {.type = MSG_APPLY_LEMTONE,
                      .u.lemtone = {.a = a, .b = b, .lm = lm, .f = fidelity}};
    return xQueueSend(s_queue, &msg, 0) == pdTRUE;
}

void controller_notify_keepalive(void) {
    if (s_queue == NULL) {
        return;
    }
    ctrl_msg_t msg = {.type = MSG_KEEPALIVE};
    xQueueSend(s_queue, &msg, 0);  // a missed keep-alive is covered by the next repeat
}

void controller_notify_dfu2(const dfu2_params_t *params) {
    if (s_queue == NULL || params == NULL) {
        return;
    }
    ctrl_msg_t msg = {.type = MSG_DFU2};
    msg.u.dfu2 = *params;
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
