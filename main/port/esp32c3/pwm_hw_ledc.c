// pwm_hw_ledc.c -- ESP32-C3 hardware PWM (LEDC) for the variant's hw_pwm channels.
//
// One low-speed timer at VARIANT_HW_PWM_FREQ_HZ / VARIANT_HW_PWM_BITS, clocked from the 80 MHz
// APB so the divider is exact (see variant_c3_rgb_cct.h), and one LEDC channel per
// hw_pwm entry of the table, in table order.

#include "pwm_hw.h"

#if VARIANT_HW_PWM

#include <math.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "pwm_hw";

#define HW_MODE  LEDC_LOW_SPEED_MODE
#define HW_TIMER LEDC_TIMER_0
#define HW_FULL  (1u << VARIANT_HW_PWM_BITS)   // duty == HW_FULL is 100 % (LEDC: [0, 2^bits])

static int8_t   s_ledc[PWM_MAX_CHANNELS];   // table index -> LEDC channel, -1 = not hw
static uint32_t s_counts[PWM_MAX_CHANNELS]; // duty currently set, in counts

void pwm_hw_init(const pwm_channel_t *ch, int n) {
    const ledc_timer_config_t timer = {
        .speed_mode = HW_MODE,
        .duty_resolution = (ledc_timer_bit_t)VARIANT_HW_PWM_BITS,
        .timer_num = HW_TIMER,
        .freq_hz = VARIANT_HW_PWM_FREQ_HZ,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    int next = 0;
    for (int i = 0; i < n && i < PWM_MAX_CHANNELS; i++) {
        s_ledc[i] = -1;
        s_counts[i] = 0;
        if (!ch[i].hw_pwm) continue;
        const ledc_channel_config_t c = {
            .gpio_num = (int)ch[i].gpio,
            .speed_mode = HW_MODE,
            .channel = (ledc_channel_t)next,
            .timer_sel = HW_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
        s_ledc[i] = (int8_t)next++;
    }
    ESP_LOGI(TAG, "LEDC: %d channel(s), %u bits at %u Hz (asked), APB clock", next,
             (unsigned)VARIANT_HW_PWM_BITS, (unsigned)ledc_get_freq(HW_MODE, HW_TIMER));
}

void pwm_hw_set(int i, float duty) {
    if (i < 0 || i >= PWM_MAX_CHANNELS || s_ledc[i] < 0) return;
    if (!(duty > 0.0f)) duty = 0.0f;  // also catches NaN
    if (duty > 1.0f) duty = 1.0f;
    const uint32_t counts = (uint32_t)lroundf(duty * (float)HW_FULL);
    if (counts == s_counts[i]) return;
    const ledc_channel_t c = (ledc_channel_t)s_ledc[i];
    if (ledc_set_duty(HW_MODE, c, counts) == ESP_OK && ledc_update_duty(HW_MODE, c) == ESP_OK) {
        s_counts[i] = counts;
    } else {
        ESP_LOGW(TAG, "LEDC channel %d: duty %u rejected", (int)c, (unsigned)counts);
    }
}

#endif  // VARIANT_HW_PWM
