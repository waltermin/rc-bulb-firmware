// pwm_output.c — 5-channel LED PWM on a custom, self-contained software engine.
//
// Each channel gets its own duty, phase, and PWM period. Periods are powers of two
// (in 200 ns ticks) so channels are harmonics and stack into one repeatable edge
// table (compiled by pwm_schedule.c). An FRC1 hardware-timer ISR walks that table:
// it is IRAM-resident and touches only RAM + GPIO/timer registers, so PWM keeps
// running through OTA flash writes and needs no radio to start.
//
// The pipeline: pwm_output_set() clamps the levels, applies the duty-response
// curve + max-power cap (unchanged from the old driver), builds one request per
// channel, compiles a schedule, and publishes it to the ISR via a lock-free double
// buffer. To try a different curve, change levels_to_duties(); to try a different
// edge/scheduling model, change pwm_schedule.c.

#include "pwm_output.h"
#include "pwm_schedule.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/hw_timer.h"
#include "driver/gpio.h"
#include "driver/soc.h"            // soc_get_ccount()
#include "esp_clk.h"               // esp_clk_cpu_freq()
#include "esp8266/gpio_struct.h"   // GPIO.out_w1ts / out_w1tc
#include "esp8266/timer_struct.h"  // frc1
#include "esp_attr.h"              // IRAM_ATTR / DRAM_ATTR
#include "esp_log.h"

#include "config.h"
#include "bulb_config.h"

static const char *TAG = "pwm";

// ---- channel table ----------------------------------------------------------

// Logical color components — the five arguments of pwm_output_set(), fixed order.
enum {
    COLOR_R = 0,
    COLOR_G,
    COLOR_B,
    COLOR_WW,
    COLOR_CW,
    COLOR_COUNT,
};

// A 5-tuple of normalized floats, indexed by COLOR_*. Used for both input levels
// and computed duty cycles as they flow through the (swappable) duty math.
typedef struct {
    float ch[COLOR_COUNT];
} color5_t;

// One physical PWM channel: its GPIO, which color feeds it, its PWM period (as a
// power-of-two tick exponent), and its base phase in degrees. Cold-white is listed
// first (channel 0, the phase reference). On the bench devboard (OMIT_I2C_PINS)
// GPIO12/14 are borrowed for I2C, so green+blue are omitted and every count below
// adjusts automatically.
typedef struct {
    uint32_t gpio;
    uint8_t  source;       // COLOR_* index feeding this channel
    uint8_t  period_log2;  // n; PWM period = 2^n ticks
    float    base_phase;   // degrees, (-180, 180]
} pwm_channel_t;

static const pwm_channel_t s_channels[] = {
#if OMIT_I2C_PINS
    { PWM_GPIO_CW,  COLOR_CW, PWM_PERIOD_LOG2_CW,  PWM_PHASE_CW  },
    { PWM_GPIO_RED, COLOR_R,  PWM_PERIOD_LOG2_RED, PWM_PHASE_RED },
    { PWM_GPIO_WW,  COLOR_WW, PWM_PERIOD_LOG2_WW,  PWM_PHASE_WW  },
#else
    { PWM_GPIO_CW,    COLOR_CW, PWM_PERIOD_LOG2_CW,    PWM_PHASE_CW    },
    { PWM_GPIO_RED,   COLOR_R,  PWM_PERIOD_LOG2_RED,   PWM_PHASE_RED   },
    { PWM_GPIO_GREEN, COLOR_G,  PWM_PERIOD_LOG2_GREEN, PWM_PHASE_GREEN },
    { PWM_GPIO_BLUE,  COLOR_B,  PWM_PERIOD_LOG2_BLUE,  PWM_PHASE_BLUE  },
    { PWM_GPIO_WW,    COLOR_WW, PWM_PERIOD_LOG2_WW,    PWM_PHASE_WW    },
#endif
};

#define PWM_CHANNELS ((int)(sizeof(s_channels) / sizeof(s_channels[0])))
_Static_assert(PWM_CHANNELS <= PWM_MAX_CHANNELS, "more channels than PWM_MAX_CHANNELS");

// ---- duty math (unchanged from the previous driver) -------------------------
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
        return t * t * t;                  // cube — no powf needed
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

// Absolute CPU-cycle (CCOUNT) time at which the edge the ISR is about to apply
// (s_idx) should fire. Advanced by each edge's exact tick gap so the frequency
// stays locked to the CPU clock, but re-anchored to "now" whenever an edge is
// applied late (bounding it to within one edge of real time in both directions —
// no accumulating drift, no runaway). Only the ISR writes it after startup
// (engine_first_arm seeds it), so no lock.
static uint32_t s_target;

// log2(CPU cycles per 200 ns tick): 4 at 80 MHz, 5 at 160 MHz. Detected once at
// init from the real CPU frequency (see pwm_output_init). Used as a shift so the
// tick<->cycle conversion in the ISR needs no divide (the LX106 has none, and a
// libgcc divide is flash-resident — unsafe in the OTA-safe ISR).
static uint32_t s_cyc_shift;

// ---- FRC1 ISR ---------------------------------------------------------------
// Applies each edge, placing the GPIO toggle at its exact time to keep PWM jitter-
// free, but bounded on every axis so it can never run away:
//   * ARM is RELATIVE and schedule-derived: gap - PWM_AHEAD_TICKS ticks (gap is the
//     tick delta to the next edge). Always in [1, length]; never wall-clock-derived,
//     so never 0 and never huge. This is what makes the timer pacing robust.
//   * SPIN is short and HARD-CAPPED: we fire ~PWM_AHEAD_TICKS early and spin on
//     CCOUNT to the edge's exact target, but bail after PWM_SPIN_CAP_TICKS so the
//     ISR can never hold the CPU for long (no watchdog).
//   * TARGET is re-anchored: it advances by exact tick gaps (locking the frequency
//     to the CPU clock), but snaps to "now" if an edge is applied late by more than
//     the spin window — bounding it to within one edge of real time, both ways.
// Touches only RAM + GPIO/FRC1 regs + CCOUNT — OTA-safe.
static void IRAM_ATTR pwm_isr(void *arg) {
    (void)arg;
    const pwm_schedule_t *sch = &s_sched[s_active];
    uint16_t idx = s_idx;
    int coalesced = 0;  // edges handled inline this invocation (dwell guard)

    const int32_t ahead = (int32_t)(PWM_AHEAD_TICKS << s_cyc_shift);
    const uint32_t spin_cap = (uint32_t)(PWM_SPIN_CAP_TICKS << s_cyc_shift);

    for (;;) {
        // Spin to this edge's exact target, then toggle. We were armed to fire a
        // little early, so the spin is short; a hard cap guarantees it can't hang if
        // the target is ever bad, and if we arrived late the test is already true so
        // we apply immediately.
        const uint32_t spin_start = soc_get_ccount();
        while ((int32_t)(soc_get_ccount() - s_target) < 0) {
            if ((soc_get_ccount() - spin_start) >= spin_cap) break;  // safety valve
        }

        const uint8_t st = PWM_EDGE_STATE(sch->edges[idx]);
        GPIO.out_w1ts = s_set_lut[st];
        GPIO.out_w1tc = s_clr_lut[st];

        // Re-anchor: if this edge was applied late by more than the spin window (the
        // ISR was starved, e.g. by phy_init), catch the timeline up to now so the
        // lateness doesn't propagate. Normal small overruns are left alone so the
        // target stays exactly on the tick grid (frequency locked).
        if ((int32_t)(soc_get_ccount() - s_target) > ahead) {
            s_target = soc_get_ccount();
        }

        // Gap (tick delta) to the next edge, and advance the absolute target by it.
        const uint32_t cur_off = PWM_EDGE_OFFSET(sch->edges[idx]);
        uint16_t next_idx = idx + 1;
        uint32_t gap;
        if (next_idx < sch->count) {
            gap = PWM_EDGE_OFFSET(sch->edges[next_idx]) - cur_off;
        } else {
            // Cycle wrap: swap in a freshly compiled schedule if one is waiting.
            gap = sch->length_ticks - cur_off;
            next_idx = 0;
            if (s_update_pending) {
                s_active ^= 1;
                s_update_pending = false;
                sch = &s_sched[s_active];
            }
        }
        idx = next_idx;
        s_target += gap << s_cyc_shift;

        // If the next edge is within the arm-ahead window, spin through it inline
        // (bounded by the coalesce cap) rather than take another interrupt; else arm
        // the one-shot to fire ~PWM_AHEAD_TICKS before it. The arm is always a sane
        // relative delay. The trampoline already cleared en; load.data then en=1
        // loads the new count and counts down.
        if (gap <= PWM_AHEAD_TICKS && coalesced < PWM_MAX_COALESCE) {
            coalesced++;
            continue;
        }
        s_idx = idx;
        frc1.load.data = (gap > PWM_AHEAD_TICKS) ? (gap - PWM_AHEAD_TICKS) : 1u;
        frc1.ctrl.en = 1;
        return;
    }
}

// ---- engine plumbing --------------------------------------------------------

static void build_luts(void) {
    uint32_t chan_mask[PWM_CHANNELS];
    s_all_pins = 0;
    for (int i = 0; i < PWM_CHANNELS; i++) {
        chan_mask[i] = 1u << s_channels[i].gpio;
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

// Build one compile request per physical channel from the computed duties.
static void build_reqs(color5_t duties, pwm_chan_req_t reqs[PWM_CHANNELS]) {
    for (int i = 0; i < PWM_CHANNELS; i++) {
        reqs[i].duty = duties.ch[s_channels[i].source];
        // Degrees (-180,180] -> fraction [0,1) of this channel's own period.
        float f = s_channels[i].base_phase / 360.0f;
        if (f < 0.0f) f += 1.0f;
        reqs[i].phase = f;
        reqs[i].period_log2 = s_channels[i].period_log2;
    }
}

// Apply the t=0 entry of the active schedule now, seed the absolute target for the
// next edge, and arm the timer (relative) to fire ~PWM_AHEAD_TICKS before it. Called
// only from a critical section while the timer is idle (init).
static void engine_first_arm(void) {
    const pwm_schedule_t *sch = &s_sched[s_active];
    const uint32_t now = soc_get_ccount();  // this edge (entry 0) is applied now
    const uint8_t st0 = PWM_EDGE_STATE(sch->edges[0]);
    GPIO.out_w1ts = s_set_lut[st0];
    GPIO.out_w1tc = s_clr_lut[st0];

    // Gap (tick delta) to the next edge: entry 1, or the wrap for a static schedule.
    uint32_t gap;
    if (sch->count > 1) {
        gap = PWM_EDGE_OFFSET(sch->edges[1]);  // entry 0 offset is 0
        s_idx = 1;
    } else {
        gap = sch->length_ticks;  // static: next event is the wrap
        s_idx = 0;
    }
    s_target = now + (gap << s_cyc_shift);
    frc1.load.data = (gap > PWM_AHEAD_TICKS) ? (gap - PWM_AHEAD_TICKS) : 1u;
    frc1.ctrl.en = 1;
    s_running = true;
}

// The per-channel request set most recently compiled into a schedule — i.e. what
// the engine is currently playing (or is staged to play at the next wrap). Used to
// skip redundant recompiles when the same duties/periods arrive again.
static pwm_chan_req_t s_last_reqs[PWM_CHANNELS];
static bool           s_have_last_reqs;

static bool reqs_equal(const pwm_chan_req_t *a, const pwm_chan_req_t *b) {
    for (int i = 0; i < PWM_CHANNELS; i++) {
        // Duties/phases are recomputed deterministically from the same inputs, so
        // an identical configuration yields bit-identical floats (no NaNs reach
        // here — the curve/clamp map them to 0), making exact compare correct.
        if (a[i].duty != b[i].duty || a[i].phase != b[i].phase ||
            a[i].period_log2 != b[i].period_log2) {
            return false;
        }
    }
    return true;
}

// Compile per-channel requests into a schedule and publish it to the ISR. Single
// producer (controller/dfu2 task, or init): while we compile, s_update_pending is
// false, so the ISR never swaps s_active out from under us and the inactive buffer
// is stable.
static void publish_reqs(const pwm_chan_req_t *reqs) {
    // Skip the (heavy) recompile + republish when the requested duty/phase/period
    // is identical to what the engine is already playing or has staged.
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

    portENTER_CRITICAL();
    if (s_running) {
        s_update_pending = true;  // ISR swaps to `target` at the next wrap
    } else {
        s_active = target;
        s_update_pending = false;
        engine_first_arm();
    }
    portEXIT_CRITICAL();
}

// Curve-mapped path: computed duties with each channel's configured phase+period.
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

void pwm_output_set_raw(const float duties[COLOR_COUNT],
                        const uint8_t period_log2[COLOR_COUNT]) {
    // Raw path: clamp duties to [0,1] but skip the curve and max-power cap, and
    // take each channel's period straight from the packet (clamped to range).
    pwm_chan_req_t reqs[PWM_CHANNELS];
    for (int i = 0; i < PWM_CHANNELS; i++) {
        const int src = s_channels[i].source;
        reqs[i].duty = clamp01(duties[src]);

        float f = s_channels[i].base_phase / 360.0f;  // degrees -> [0,1) fraction
        if (f < 0.0f) f += 1.0f;
        reqs[i].phase = f;

        uint8_t n = period_log2[src];
        if (n < PWM_PERIOD_LOG2_MIN) n = PWM_PERIOD_LOG2_MIN;
        else if (n > PWM_PERIOD_LOG2_MAX) n = PWM_PERIOD_LOG2_MAX;
        reqs[i].period_log2 = n;
    }
    publish_reqs(reqs);
}

void pwm_output_init(void) {
    // Detect the CPU->tick shift from the real CPU frequency (this SDK defaults to
    // 160 MHz -> 32 cycles/tick -> shift 5; 80 MHz -> 16 -> shift 4). Must be set
    // before the ISR ever runs. esp_clk_cpu_freq() is a couple of global reads, so
    // this costs nothing at cold start. 200 ns/tick, so cycles/tick = MHz / 5.
    const uint32_t cpt = ((uint32_t)esp_clk_cpu_freq() / 1000000u * PWM_TICK_NS) / 1000u;
    s_cyc_shift = 0;
    while ((1u << s_cyc_shift) < cpt) s_cyc_shift++;  // log2(cpt); 16->4, 32->5

    build_luts();

    // Configure every channel pin as a low output before driving PWM.
    gpio_config_t io = {
        .pin_bit_mask = s_all_pins,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    GPIO.out_w1tc = s_all_pins;  // all low

    // FRC1 at clkdiv16 -> 200 ns/tick, one-shot, edge interrupt. hw_timer_init
    // registers our ISR and unmasks the line; the timer stays disarmed until
    // engine_first_arm() sets the compare and enables it. NOTE: hw_timer_disarm()
    // zeroes the whole control register (clkdiv/reload/intr included), so it must
    // run BEFORE we set those — otherwise the divider reverts to clkdiv1 (80 MHz)
    // and every tick would be 16x too short.
    ESP_ERROR_CHECK(hw_timer_init(pwm_isr, NULL));
    hw_timer_disarm();
    hw_timer_set_clkdiv(TIMER_CLKDIV_16);
    hw_timer_set_reload(false);
    hw_timer_set_intr_type(TIMER_EDGE_INT);

    s_active = 0;
    s_running = false;
    s_update_pending = false;
    s_idx = 0;

    // Compile + light the default color now — no radio needed (FRC1 ticks from boot).
    pwm_output_set_default();

    ESP_LOGI(TAG, "pwm init: %d channels, %d ns/tick, cpu %d MHz -> cyc_shift %u, "
             "default period 2^%d ticks%s",
             PWM_CHANNELS, PWM_TICK_NS, esp_clk_cpu_freq() / 1000000,
             (unsigned)s_cyc_shift, PWM_DEFAULT_PERIOD_LOG2,
             OMIT_I2C_PINS ? " (green+blue omitted: GPIO12/14 left for I2C)" : "");
}
