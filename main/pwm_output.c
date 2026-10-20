// pwm_output.c -- 5-channel LED PWM on a custom, self-contained software engine.
//
// Each channel gets its own duty, phase, and PWM period. Periods are powers of two
// (in 1 us ticks) so channels are harmonics and stack into one repeatable edge
// table (compiled by pwm_schedule.c). An IRAM-resident ISR, running above every
// critical section in the system, walks that table: it touches only RAM + GPIO and
// timer registers, so it stays live through OTA flash writes.
//
// All hardware lives behind three seams:
//   port/pwm_timer.h     reading time, arming the next edge, GPIO writes
//   port/pwm_hw.h        channels a PWM peripheral drives instead (variant hw_pwm)
//   variants/variant.h   which GPIO each channel drives and what it MEANS
//
// The pipeline: pwm_output_set() clamps the levels, applies the duty-response
// curve + max-power cap, mixes logical colour onto the variant's physical channels
// (pwm_mix.c), builds one request per channel, compiles a schedule, and publishes
// it to the ISR via a lock-free double buffer. To try a different curve, change
// levels_to_duties(); a different edge model, pwm_schedule.c; a different board,
// a variant header.

#include "pwm_output.h"
#include "pwm_schedule.h"
#include "pwm_mix.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "pwm_hw.h"
#include "pwm_timer.h"
#include "variant.h"

#include "config.h"
#include "bulb_config.h"

static const char *TAG = "pwm";

// The variant header supplies s_channels[]; everything below sizes off it.
#define PWM_CHANNELS ((int)(sizeof(s_channels) / sizeof(s_channels[0])))
_Static_assert(PWM_CHANNELS <= PWM_MAX_CHANNELS, "more channels than PWM_MAX_CHANNELS");

// Arm-ahead window and spin budget, expressed in the backend's native ticks.
#define PWM_AHEAD_TICKS PWM_US_TO_TICKS(PWM_AHEAD_US)

// ---- duty math --------------------------------------------------------------
// clamp -> levels_to_duties (curve) -> clamp -> scale by max power. Pure float,
// no engine/SDK dependency; the single swap point for a new duty curve.

static inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static inline color5_t color5_clamp01(color5_t c) {
    for (int i = 0; i < COLOR_COUNT; i++) {
        c.ch[i] = clamp01(c.ch[i]);
    }
    return c;
}

// Power-law gamma: duty = level ^ gamma (identity when gamma == 1.0f).
static inline float gamma_to_duty(float x, float gamma) {
    return powf(x, gamma);
}

// Perceptual (CIE L*) -> normalized duty cycle. Linear toe near black, then cube.
static inline float perceptual_to_duty(float x) {
    if (x <= 0.08f) {
        return x / 9.033f;                 // linear toe: hits exactly 0 at x=0
    } else {
        float t = (x + 0.16f) / 1.16f;     // = (100x + 16) / 116
        return t * t * t;                  // cube -- no powf needed
    }
}

// Normalized levels -> normalized duty cycles, applying the configured curve.
static color5_t levels_to_duties(color5_t levels) {
    const uint8_t curve = bulb_config_get_u8(CFG_DUTY_CURVE);
    const float   gamma = bulb_config_get_float(CFG_GAMMA);
    color5_t duties;
    for (int i = 0; i < COLOR_COUNT; i++) {
        duties.ch[i] = (curve == PWM_CURVE_PERCEPTUAL)
                           ? perceptual_to_duty(levels.ch[i])
                           : gamma_to_duty(levels.ch[i], gamma);
    }
    return duties;
}

// Full level -> duty transform: clamp, curve, clamp, scale by the max-power cap.
// The mixer runs strictly AFTER this, which is what lets a CCT channel stay a pure
// ratio -- both the curve and the cap cancel out of it. See pwm_mix.c.
static color5_t compute_duties(float r, float g, float b, float ww, float cw) {
    color5_t levels = color5_clamp01((color5_t){{r, g, b, ww, cw}});
    color5_t duties = color5_clamp01(levels_to_duties(levels));
    for (int i = 0; i < COLOR_COUNT; i++) {
        duties.ch[i] *= PWM_MAX_POWER;
    }
    return duties;
}

// ---- engine state -----------------------------------------------------------

// state-byte (bit i == channel i high) -> GPIO set/clear words. Built once at init
// from the fixed channel->GPIO map; never changes. DRAM so the IRAM ISR can read
// them with the flash cache disabled.
static DRAM_ATTR uint32_t s_set_lut[256];
static DRAM_ATTR uint32_t s_clr_lut[256];
static uint32_t s_all_pins;  // union of every channel's GPIO mask

// Double-buffered compiled schedules + the lock-free handoff flags. The ISR plays
// s_sched[s_active]; a new schedule is compiled into the inactive buffer and picked
// up at the next cycle wrap when s_update_pending is set.
static DRAM_ATTR pwm_schedule_t s_sched[2];
static volatile uint8_t  s_active;          // buffer the ISR is currently playing
static volatile bool     s_update_pending;  // inactive buffer holds a newer schedule
static volatile uint16_t s_idx;             // next entry the ISR will apply
static volatile bool     s_running;         // timer armed / engine started

// Timer tick at which the ISR should apply the edge s_idx points at, in the
// CURRENT epoch. The ISR advances it by each edge's gap, then hands it to
// pwm_tmr_arm_rebase(), which returns it re-expressed in whatever epoch exists
// after arming (see port/pwm_timer.h). Only the ISR writes it after startup.
static uint32_t s_target;

// ---- edge ISR ---------------------------------------------------------------
// Applies each edge at its exact tick:
//   * SPIN is short and HARD-CAPPED: we wake PWM_AHEAD_US early and spin on the
//     live counter to the edge's exact tick, bailing after PWM_SPIN_MAX_ITERS so
//     the ISR can never hold the CPU for long. If we arrived late the test is
//     already false, so we apply immediately.
//   * TARGET is re-based every arm, so an edge that lands late catches the
//     timeline up rather than dragging the lateness forward.
// Touches only RAM + GPIO/timer registers -- OTA-safe, and safe to run inside
// someone else's critical section (which, on the C3, it does).
static void IRAM_ATTR pwm_isr(void) {
    pwm_tmr_ack();

    const pwm_schedule_t *sch = &s_sched[s_active];
    uint16_t idx = s_idx;
    int coalesced = 0;  // edges handled inline this invocation (dwell guard)

    for (;;) {
        uint32_t spin_iters = 0;
        while ((int32_t)(pwm_tmr_now() - s_target) < 0) {
            if (++spin_iters >= PWM_SPIN_MAX_ITERS) break;  // safety valve
        }

        const uint8_t st = PWM_EDGE_STATE(sch->edges[idx]);
        pwm_gpio_set(s_set_lut[st]);
        pwm_gpio_clr(s_clr_lut[st]);

        // Advance to the next edge and grow the target by the gap to it.
        const uint32_t cur_us = PWM_EDGE_OFFSET(sch->edges[idx]);
        uint16_t next_idx = idx + 1;
        uint32_t next_us;
        if (next_idx < sch->count) {
            next_us = PWM_EDGE_OFFSET(sch->edges[next_idx]);
        } else {
            // Cycle wrap: swap in a freshly compiled schedule if one is waiting.
            next_us = sch->length_ticks;
            next_idx = 0;
            if (s_update_pending) {
                s_active ^= 1;
                s_update_pending = false;
                sch = &s_sched[s_active];
            }
        }
        idx = next_idx;
        s_target += PWM_US_TO_TICKS(next_us - cur_us);

        // If the next edge is within the arm-ahead window, spin through it inline
        // (bounded by the coalesce cap) rather than take another interrupt.
        if ((int32_t)(s_target - pwm_tmr_now()) <= (int32_t)PWM_AHEAD_TICKS
            && coalesced < PWM_MAX_COALESCE) {
            coalesced++;
            continue;
        }
        break;
    }

    s_idx = idx;
    s_target = pwm_tmr_arm_rebase(s_target, PWM_AHEAD_TICKS);
}

// ---- engine plumbing --------------------------------------------------------

static void build_luts(void) {
    uint32_t chan_mask[PWM_CHANNELS];
    s_all_pins = 0;
    for (int i = 0; i < PWM_CHANNELS; i++) {
        // hw_pwm channels belong to the peripheral: the ISR never touches their pins
        chan_mask[i] = s_channels[i].hw_pwm ? 0u : 1u << s_channels[i].gpio;
        s_all_pins |= chan_mask[i];
    }
    for (int state = 0; state < 256; state++) {
        uint32_t set = 0;
        for (int i = 0; i < PWM_CHANNELS; i++) {
            if (state & (1 << i)) set |= chan_mask[i];
        }
        s_set_lut[state] = set;
        s_clr_lut[state] = s_all_pins & ~set;
    }
}

// Degrees (-180,180] -> fraction [0,1) of this channel's own period.
static inline float phase_fraction(float degrees) {
    float f = degrees / 360.0f;
    if (f < 0.0f) f += 1.0f;
    return f;
}

// Hand the hw_pwm channels' duties to the peripheral and leave them out of the schedule
// (duty 0: the engine compiles no edges for them). The peripheral switches at its next
// carrier period, the schedule at its next wrap: at most one long period apart.
static void route_hw(const float mixed[PWM_CHANNELS], pwm_chan_req_t reqs[PWM_CHANNELS]) {
#if VARIANT_HW_PWM
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_channels[i].hw_pwm) {
            pwm_hw_set(i, mixed[i]);
            reqs[i].duty = 0.0f;
        }
    }
#else
    (void)mixed;
    (void)reqs;
#endif
}

// Build one compile request per physical channel from the computed duties, mixing
// logical colour onto whatever this board actually has.
static void build_reqs(color5_t duties, pwm_chan_req_t reqs[PWM_CHANNELS]) {
    float mixed[PWM_CHANNELS];
    pwm_mix(duties, s_channels, PWM_CHANNELS, mixed);
    for (int i = 0; i < PWM_CHANNELS; i++) {
        reqs[i].duty        = mixed[i];
        reqs[i].phase       = phase_fraction(s_channels[i].base_phase);
        reqs[i].period_log2 = s_channels[i].period_log2;
    }
    route_hw(mixed, reqs);
}

// Apply the t=0 entry of the active schedule now, seed the target for the next
// edge, and arm the timer. Called only from a critical section while the timer is
// idle (init).
static void engine_first_arm(void) {
    const pwm_schedule_t *sch = &s_sched[s_active];
    const uint8_t st0 = PWM_EDGE_STATE(sch->edges[0]);
    pwm_gpio_set(s_set_lut[st0]);
    pwm_gpio_clr(s_clr_lut[st0]);

    // Ticks to the next edge: entry 1, or the wrap for a static schedule.
    uint32_t gap_us;
    if (sch->count > 1) {
        gap_us = PWM_EDGE_OFFSET(sch->edges[1]);
        s_idx = 1;
    } else {
        gap_us = sch->length_ticks;  // static: next event is the wrap
        s_idx = 0;
    }
    s_target = pwm_tmr_now() + PWM_US_TO_TICKS(gap_us);
    s_target = pwm_tmr_arm_rebase(s_target, PWM_AHEAD_TICKS);
    s_running = true;
}

// The per-channel request set most recently compiled -- i.e. what the engine is
// playing (or is staged to play at the next wrap). Used to skip redundant
// recompiles when the same duties/periods arrive again.
static pwm_chan_req_t s_last_reqs[PWM_CHANNELS];
static bool           s_have_last_reqs;

static bool reqs_equal(const pwm_chan_req_t *a, const pwm_chan_req_t *b) {
    for (int i = 0; i < PWM_CHANNELS; i++) {
        // Duties/phases are recomputed deterministically from the same inputs, so
        // an identical configuration yields bit-identical floats (no NaNs reach
        // here -- the curve/clamp map them to 0), making exact compare correct.
        if (a[i].duty != b[i].duty || a[i].phase != b[i].phase ||
            a[i].period_log2 != b[i].period_log2) {
            return false;
        }
    }
    return true;
}

// Compile per-channel requests into a schedule and publish it to the ISR.
//
// Single producer (controller/dfu2 task, or init): while we compile,
// s_update_pending is false, so the ISR never swaps s_active out from under us and
// the inactive buffer is stable. That invariant -- not the critical section below
// -- is what makes the handoff safe, which matters because on the C3 the ISR runs
// at priority 4 and PWM_ENTER_CRITICAL does NOT hold it off.
static void publish_reqs(const pwm_chan_req_t *reqs) {
    if (s_have_last_reqs && reqs_equal(reqs, s_last_reqs)) {
        return;
    }

    const uint8_t target = s_running ? (uint8_t)(s_active ^ 1) : s_active;
    if (!pwm_compile(reqs, PWM_CHANNELS, &s_sched[target])) {
        ESP_LOGW(TAG, "pwm_compile overflow; keeping previous frame");
        return;
    }
    memcpy(s_last_reqs, reqs, sizeof s_last_reqs);
    s_have_last_reqs = true;

    PWM_ENTER_CRITICAL();
    if (s_running) {
        s_update_pending = true;  // ISR swaps to `target` at the next wrap
    } else {
        s_active = target;
        s_update_pending = false;
        engine_first_arm();
    }
    PWM_EXIT_CRITICAL();
}

static void publish_duties(color5_t duties) {
    pwm_chan_req_t reqs[PWM_CHANNELS];
    build_reqs(duties, reqs);
    publish_reqs(reqs);
}

// ---- public API -------------------------------------------------------------

void pwm_output_set(float r, float g, float b, float ww, float cw) {
    publish_duties(compute_duties(r, g, b, ww, cw));
}

void pwm_output_set_default(void) {
    publish_duties(compute_duties(bulb_config_get_float(CFG_DEFAULT_R),
                                  bulb_config_get_float(CFG_DEFAULT_G),
                                  bulb_config_get_float(CFG_DEFAULT_B),
                                  bulb_config_get_float(CFG_DEFAULT_WW),
                                  bulb_config_get_float(CFG_DEFAULT_CW)));
}

unsigned pwm_output_hw_bits(void) {
#if VARIANT_HW_PWM
    return VARIANT_HW_PWM_BITS;
#else
    return 0;
#endif
}

void pwm_output_set_raw(const float duties[COLOR_COUNT],
                        const uint8_t period_log2[COLOR_COUNT]) {
    // Raw path: clamp duties to [0,1] but skip the curve and max-power cap, and
    // take each channel's period from the packet. Still goes through the mixer, so
    // "raw" means the same thing on every variant: raw LOGICAL duties.
    color5_t d;
    for (int i = 0; i < COLOR_COUNT; i++) {
        d.ch[i] = clamp01(duties[i]);
    }
    float mixed[PWM_CHANNELS];
    pwm_mix(d, s_channels, PWM_CHANNELS, mixed);

    pwm_chan_req_t reqs[PWM_CHANNELS];
    for (int i = 0; i < PWM_CHANNELS; i++) {
        reqs[i].duty  = mixed[i];
        reqs[i].phase = phase_fraction(s_channels[i].base_phase);

        // A driver IC reads duty as a level and wants a stable carrier, so
        // rate-locked channels keep their configured period no matter what the
        // packet asked for.
        if (s_channels[i].rate_locked) {
            reqs[i].period_log2 = s_channels[i].period_log2;
        } else {
            uint8_t n = period_log2[s_channels[i].source];
            if (n < PWM_PERIOD_LOG2_MIN) n = PWM_PERIOD_LOG2_MIN;
            else if (n > PWM_PERIOD_LOG2_MAX) n = PWM_PERIOD_LOG2_MAX;
            reqs[i].period_log2 = n;
        }
    }
    route_hw(mixed, reqs);
    publish_reqs(reqs);
}

static void start_engine(void) {
    pwm_tmr_init(pwm_isr);
    PWM_ENTER_CRITICAL();
    engine_first_arm();
    pwm_tmr_enable();   // only after the first compare is armed in the future
    PWM_EXIT_CRITICAL();
    ESP_LOGI(TAG, "pwm started (%u ticks/us, ahead %d us)",
             (unsigned)PWM_TICKS_PER_US, PWM_AHEAD_US);
}

void pwm_output_init(void) {
    build_luts();

    gpio_config_t io = {
        .pin_bit_mask = s_all_pins,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    pwm_gpio_clr(s_all_pins);  // all low
#if VARIANT_HW_PWM
    pwm_hw_init(s_channels, PWM_CHANNELS);  // at duty 0 until the default below
#else
    for (int i = 0; i < PWM_CHANNELS; i++) {
        if (s_channels[i].hw_pwm) ESP_LOGE(TAG, "channel %d is hw_pwm but the variant has no VARIANT_HW_PWM", i);
    }
#endif

    s_active = 0;
    s_running = false;
    s_update_pending = false;
    s_idx = 0;

    // Compile the default colour and drive its static t=0 level now.
    color5_t duties = compute_duties(bulb_config_get_float(CFG_DEFAULT_R),
                                     bulb_config_get_float(CFG_DEFAULT_G),
                                     bulb_config_get_float(CFG_DEFAULT_B),
                                     bulb_config_get_float(CFG_DEFAULT_WW),
                                     bulb_config_get_float(CFG_DEFAULT_CW));
    pwm_chan_req_t reqs[PWM_CHANNELS];
    build_reqs(duties, reqs);
    pwm_compile(reqs, PWM_CHANNELS, &s_sched[0]);
    memcpy(s_last_reqs, reqs, sizeof s_last_reqs);
    s_have_last_reqs = true;
    const uint8_t st0 = PWM_EDGE_STATE(s_sched[0].edges[0]);
    pwm_gpio_set(s_set_lut[st0]);
    pwm_gpio_clr(s_clr_lut[st0]);

    ESP_LOGI(TAG, "pwm init: variant %s, %d channels, default period 2^%d us",
             PWM_VARIANT_NAME, PWM_CHANNELS, PWM_DEFAULT_PERIOD_LOG2);

#if !PWM_TIMER_NEEDS_RADIO
    // systimer runs from boot, so real modulation starts immediately -- no static
    // pre-radio window as on the ESP8266.
    start_engine();
#endif
}

void pwm_output_start_after_radio(void) {
#if PWM_TIMER_NEEDS_RADIO
    // The WDEV/TSF0 timer only ticks now that esp_wifi_start() brought the radio
    // up; until this point the LED has shown the default's static level.
    start_engine();
#endif
}
