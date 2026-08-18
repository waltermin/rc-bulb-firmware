// pwm_output.c — 5-channel LED PWM using the ESP8266_RTOS_SDK software PWM
// driver. The driver's timer ISR is IRAM-resident, so PWM keeps running safely
// even while OTA flash writes briefly disable the flash cache.

#include "pwm_output.h"

#include <math.h>

#include "driver/pwm.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "pwm";

// Duty values are in PWM timer ticks, capped at PWM_PERIOD_US.
static uint32_t s_duties[PWM_CHANNELS];

// Pin list indexed the same as the duty array (see PWM_IDX_* in config.h).
static const uint32_t s_pins[PWM_CHANNELS] = {
    [PWM_IDX_CW] = PWM_GPIO_CW,
    [PWM_IDX_RED] = PWM_GPIO_RED,
    [PWM_IDX_GREEN] = PWM_GPIO_GREEN,
    [PWM_IDX_BLUE] = PWM_GPIO_BLUE,
    [PWM_IDX_WW] = PWM_GPIO_WW,
};

static float s_phases[PWM_CHANNELS] = {
    [PWM_IDX_CW] = PWM_PHASE_CW,
    [PWM_IDX_RED] = PWM_PHASE_RED,
    [PWM_IDX_GREEN] = PWM_PHASE_GREEN,
    [PWM_IDX_BLUE] = PWM_PHASE_BLUE,
    [PWM_IDX_WW] = PWM_PHASE_WW,
};

// Map a u8 channel value to a PWM duty (ticks), applying gamma and the max-power
// cap exactly once. 0 -> 0, 255 -> PWM_MAX_POWER * period.
static uint32_t value_to_duty(uint8_t v) {
    if (v == 0) {
        return 0;
    }
    float norm = (float)v / 255.0f;
    norm = powf(norm, PWM_GAMMA);  // identity when PWM_GAMMA == 1.0f
    float duty = norm * PWM_MAX_POWER * (float)PWM_PERIOD_US;
    uint32_t d = (uint32_t)(duty + 0.5f);
    if (d >= PWM_PERIOD_US) {
        d = PWM_PERIOD_US - 1;  // driver requires duty < period
    }
    return d;
}

void pwm_output_init(void) {
    for (int i = 0; i < PWM_CHANNELS; i++) {
        s_duties[i] = 0;
    }
    // period in microseconds, initial duties all zero.
    pwm_init(PWM_PERIOD_US, s_duties, PWM_CHANNELS, s_pins);
    pwm_set_phases(s_phases);
    pwm_start();
    pwm_output_set_default();
    ESP_LOGI(TAG, "pwm init: %d channels @ %d Hz", PWM_CHANNELS, PWM_FREQ_HZ);
}

void pwm_output_set(uint8_t r, uint8_t g, uint8_t b, uint8_t ww, uint8_t cw) {
    s_duties[PWM_IDX_RED] = value_to_duty(r);
    s_duties[PWM_IDX_GREEN] = value_to_duty(g);
    s_duties[PWM_IDX_BLUE] = value_to_duty(b);
    s_duties[PWM_IDX_WW] = value_to_duty(ww);
    s_duties[PWM_IDX_CW] = value_to_duty(cw);
    pwm_set_duties(s_duties);
    pwm_start();  // commit the new duty table
}

void pwm_output_set_default(void) {
    pwm_output_set(DEFAULT_R, DEFAULT_G, DEFAULT_B, DEFAULT_WW, DEFAULT_CW);
}
