// pwm_output.c — 5-channel LED PWM using the ESP8266_RTOS_SDK software PWM
// driver. The driver's timer ISR is IRAM-resident, so PWM keeps running safely
// even while OTA flash writes briefly disable the flash cache.
//
// The active channel set is generated from a single X-macro list so that
// omitting channels (see OMIT_I2C_PINS in config.h) keeps the pin/phase/duty
// arrays and their indices consistent automatically.

#include "pwm_output.h"

#include <math.h>

#include "driver/pwm.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "pwm";

// X(NAME, gpio, phase_degrees, color_field)
//   NAME        -> generates channel index CH_NAME
//   color_field -> which pwm_output_set() argument feeds this channel
// Cold-white is listed first so it is index 0 and serves as the phase base.
#if OMIT_I2C_PINS
#define PWM_CHANNEL_LIST                              \
    X(CW,  PWM_GPIO_CW,  PWM_PHASE_CW,  cw)           \
    X(RED, PWM_GPIO_RED, PWM_PHASE_RED, r)            \
    X(WW,  PWM_GPIO_WW,  PWM_PHASE_WW,  ww)
#else
#define PWM_CHANNEL_LIST                              \
    X(CW,    PWM_GPIO_CW,    PWM_PHASE_CW,    cw)      \
    X(RED,   PWM_GPIO_RED,   PWM_PHASE_RED,   r)       \
    X(GREEN, PWM_GPIO_GREEN, PWM_PHASE_GREEN, g)       \
    X(BLUE,  PWM_GPIO_BLUE,  PWM_PHASE_BLUE,  b)       \
    X(WW,    PWM_GPIO_WW,    PWM_PHASE_WW,    ww)
#endif

// Channel index enum + count.
enum {
#define X(name, gpio, phase, field) CH_##name,
    PWM_CHANNEL_LIST
#undef X
    PWM_CHANNELS
};

static uint32_t s_duties[PWM_CHANNELS];

static const uint32_t s_pins[PWM_CHANNELS] = {
#define X(name, gpio, phase, field) [CH_##name] = (gpio),
    PWM_CHANNEL_LIST
#undef X
};

static float s_phases[PWM_CHANNELS] = {
#define X(name, gpio, phase, field) [CH_##name] = (phase),
    PWM_CHANNEL_LIST
#undef X
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
    ESP_LOGI(TAG, "pwm init: %d channels @ %d Hz%s", PWM_CHANNELS, PWM_FREQ_HZ,
             OMIT_I2C_PINS ? " (green+blue omitted: GPIO12/14 left for I2C)" : "");
}

void pwm_output_set(uint8_t r, uint8_t g, uint8_t b, uint8_t ww, uint8_t cw) {
    // Unused args when channels are omitted; keep the stable 5-arg interface.
    (void)r; (void)g; (void)b; (void)ww; (void)cw;
#define X(name, gpio, phase, field) s_duties[CH_##name] = value_to_duty(field);
    PWM_CHANNEL_LIST
#undef X
    pwm_set_duties(s_duties);
    pwm_start();  // commit the new duty table
}

void pwm_output_set_default(void) {
    pwm_output_set(DEFAULT_R, DEFAULT_G, DEFAULT_B, DEFAULT_WW, DEFAULT_CW);
}

void pwm_output_start_after_radio(void) {
    // The SDK PWM driver runs its ISR off the Wi-Fi MAC (WDEV/TSF0) hardware
    // timer, which only ticks after esp_wifi_start(). The pwm_start() issued in
    // pwm_output_init() therefore armed a timer that never fired, leaving the
    // LEDs dark. Re-arm now that the radio is up: pwm_stop() clears the driver's
    // internal start_flag so the pwm_start() inside pwm_output_set_default()
    // re-runs pwm_timer_start() — this time under the live WDEV clock.
    pwm_stop(0x0);              // all channels low; start_flag -> 0
    pwm_output_set_default();   // re-applies default duties and truly starts PWM
    ESP_LOGI(TAG, "pwm re-armed after radio start");
}
