// pwm_output.c — 5-channel LED PWM built on the ESP8266_RTOS_SDK software PWM
// driver. The driver's timer ISR is IRAM-resident, so PWM keeps running safely
// even while OTA flash writes briefly disable the flash cache.
//
// The drive algorithm — how the five normalized channel levels become a
// concrete PWM frame (period, per-channel duty, per-channel phase) — is
// isolated in compute_frame(). Everything else is plumbing: pwm_output_set()
// builds the level vector, compute_frame() decides the frame, and commit_frame()
// hands it to the driver. To try a different way of driving the LEDs, change
// compute_frame() (and/or level_to_duty()) and nothing else.

#include "pwm_output.h"

#include <math.h>

#include "driver/pwm.h"
#include "esp_log.h"

#include "config.h"

static const char *TAG = "pwm";

// ---- channel table ----------------------------------------------------------

// Logical color components. These are the five arguments of pwm_output_set(),
// in a fixed order; each physical channel draws its level from one of them.
enum {
    COLOR_R = 0,
    COLOR_G,
    COLOR_B,
    COLOR_WW,
    COLOR_CW,
    COLOR_COUNT,
};

// A 5-tuple of normalized floats, indexed by COLOR_*. Used for both the input
// levels and the computed duty cycles as they flow through the pipeline, so the
// swappable stage functions can take and return the whole tuple by value.
typedef struct {
    float ch[COLOR_COUNT];
} color5_t;

// One physical PWM channel: its GPIO, which color level feeds it, and its base
// phase offset in degrees. Cold-white is listed first so it is channel 0 and
// serves as the phase reference. On the bench devboard (OMIT_I2C_PINS) GPIO12
// and GPIO14 are borrowed for an I2C device, so the green and blue channels are
// simply left out of the table — their pins stay untouched and every index/count
// below adjusts automatically.
typedef struct {
    uint32_t gpio;
    uint8_t  source;      // COLOR_* index feeding this channel
    float    base_phase;  // degrees, (-180, 180]
} pwm_channel_t;

static const pwm_channel_t s_channels[] = {
#if OMIT_I2C_PINS
    { PWM_GPIO_CW,  COLOR_CW, PWM_PHASE_CW  },
    { PWM_GPIO_RED, COLOR_R,  PWM_PHASE_RED },
    { PWM_GPIO_WW,  COLOR_WW, PWM_PHASE_WW  },
#else
    { PWM_GPIO_CW,    COLOR_CW, PWM_PHASE_CW    },
    { PWM_GPIO_RED,   COLOR_R,  PWM_PHASE_RED   },
    { PWM_GPIO_GREEN, COLOR_G,  PWM_PHASE_GREEN },
    { PWM_GPIO_BLUE,  COLOR_B,  PWM_PHASE_BLUE  },
    { PWM_GPIO_WW,    COLOR_WW, PWM_PHASE_WW    },
#endif
};

#define PWM_CHANNELS ((int)(sizeof(s_channels) / sizeof(s_channels[0])))

// A fully-computed PWM frame: everything the driver needs for one update.
typedef struct {
    uint32_t period_us;
    uint32_t duties[PWM_CHANNELS];
    float    phases[PWM_CHANNELS];
} pwm_frame_t;

// ---- drive algorithm --------------------------------------------------------
// The pipeline in pwm_output_set() is: clamp -> levels_to_duties (curve) ->
// scale by max power -> duties_to_frame -> commit. Everything up to the frame
// stays in normalized float space; only duties_to_frame() converts to ticks.
// The two stage functions below are the swap points for trying new algorithms.

static inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Clamp every channel of a 5-tuple to [0,1].
static inline color5_t color5_clamp01(color5_t c) {
    for (int i = 0; i < COLOR_COUNT; i++) {
        c.ch[i] = clamp01(c.ch[i]);
    }
    return c;
}

// A duty-response curve maps a normalized channel level in [0,1] to a normalized
// duty in [0,1]. levels_to_duties() picks one via PWM_DUTY_CURVE (config.h).
// Both are defined so either can be selected without touching this file.

// Power-law gamma: duty = level ^ PWM_GAMMA (identity when PWM_GAMMA == 1.0f).
static inline float gamma_to_duty(float x) {
    return powf(x, PWM_GAMMA);
}

// Perceptual (CIE L*) -> normalized duty cycle.
// x: normalized input in [0,1]  (e.g. netval / 255.0f, or /65535.0f for 16-bit)
// returns: normalized duty in [0,1]
static inline float perceptual_to_duty(float x) {
    if (x <= 0.08f) {
        return x / 9.033f;                 // linear toe: hits exactly 0 at x=0
    } else {
        float t = (x + 0.16f) / 1.16f;     // = (100x + 16) / 116
        return t * t * t;                  // cube — no powf needed
    }
}

// STAGE 1 (swappable): normalized levels -> normalized duty cycles.
// Applies the configured duty-response curve to each channel. Because it takes
// and returns the whole 5-tuple, a replacement may also do cross-channel work
// (white balancing, gamut mapping, ...) instead of a pure per-channel curve.
static color5_t levels_to_duties(color5_t levels) {
    color5_t duties;
    for (int i = 0; i < COLOR_COUNT; i++) {
#if PWM_DUTY_CURVE == PWM_CURVE_PERCEPTUAL
        duties.ch[i] = perceptual_to_duty(levels.ch[i]);
#else
        duties.ch[i] = gamma_to_duty(levels.ch[i]);
#endif
    }
    return duties;
}

// STAGE 2 (swappable): normalized duty cycles -> a concrete PWM frame.
// Chooses the period and, for each physical channel, its duty (converted to
// timer ticks) and phase; this is also where the logical colors are fanned out
// to physical channels. Today it uses a fixed base period and the per-channel
// base phases from config, but nothing outside this function assumes those are
// constant — vary f.period_us or f.phases[i] per frame to experiment.
static pwm_frame_t duties_to_frame(color5_t duties) {
    pwm_frame_t f;
    f.period_us = PWM_PERIOD_US;  // dynamic: this is the default, free to vary

    for (int i = 0; i < PWM_CHANNELS; i++) {
        float duty = duties.ch[s_channels[i].source];
        uint32_t ticks = (uint32_t)(duty * (float)f.period_us + 0.5f);
        if (ticks >= f.period_us) {
            ticks = f.period_us - 1;  // driver requires duty < period
        }
        f.duties[i] = ticks;
        f.phases[i] = s_channels[i].base_phase;
    }
    return f;
}

// ---- driver plumbing --------------------------------------------------------

// Push a computed frame to the driver and start output. The SDK's pwm_set_*
// calls copy the values into the driver's internal state, so the frame does not
// need to outlive this call; pwm_start() commits the new period/duty/phase.
static void commit_frame(pwm_frame_t *f) {
    pwm_set_period(f->period_us);
    pwm_set_phases(f->phases);
    pwm_set_duties(f->duties);
    pwm_start();
}

void pwm_output_set(float r, float g, float b, float ww, float cw) {
    // 1. Clamp the incoming levels to [0,1].
    color5_t levels = color5_clamp01((color5_t){{r, g, b, ww, cw}});

    // 2. Levels -> duty cycles (swappable curve), all channels together.
    color5_t duties = levels_to_duties(levels);

    // 3. Clamp duties (a swapped-in curve may over/undershoot) then scale by the
    //    max-power cap, still in float space.
    duties = color5_clamp01(duties);
    for (int i = 0; i < COLOR_COUNT; i++) {
        duties.ch[i] *= PWM_MAX_POWER;
    }

    // 4. Duty cycles -> PWM frame (swappable), all channels together.
    pwm_frame_t frame = duties_to_frame(duties);

    // 5. Apply the frame.
    commit_frame(&frame);
}

void pwm_output_set_default(void) {
    pwm_output_set(DEFAULT_R, DEFAULT_G, DEFAULT_B, DEFAULT_WW, DEFAULT_CW);
}

void pwm_output_init(void) {
    // pwm_init/pwm_set_phases copy their arrays, so these locals are enough.
    uint32_t pins[PWM_CHANNELS];
    uint32_t duties[PWM_CHANNELS];
    float    phases[PWM_CHANNELS];
    for (int i = 0; i < PWM_CHANNELS; i++) {
        pins[i]   = s_channels[i].gpio;
        duties[i] = 0;  // start dark; pwm_output_set_default() lights up below
        phases[i] = s_channels[i].base_phase;
    }

    pwm_init(PWM_PERIOD_US, duties, PWM_CHANNELS, pins);
    pwm_set_phases(phases);
    pwm_start();
    pwm_output_set_default();
    ESP_LOGI(TAG, "pwm init: %d channels @ %d Hz%s", PWM_CHANNELS, PWM_FREQ_HZ,
             OMIT_I2C_PINS ? " (green+blue omitted: GPIO12/14 left for I2C)" : "");
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
