// lemtone_wasm.c — WebAssembly exports of the Lemtone solver for lemtone_tool.html.
//
// Freestanding (no libc): build.py compiles this with lemtone.c and the generated
// model into one wasm32 module. Results live in statics read back through getters,
// so the JS side never has to know struct layouts.

#include <stddef.h>

#include "lemtone.h"

#define EXPORT(name) __attribute__((export_name(#name)))

// The compiler may lower struct copies to these.
void *memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = dst;
    const unsigned char *s = src;
    while (n--) *d++ = *s++;
    return dst;
}
void *memset(void *dst, int c, size_t n) {
    unsigned char *d = dst;
    while (n--) *d++ = (unsigned char)c;
    return dst;
}

static lemtone_ctx_t        s_ctx;
static lemtone_period_cfg_t s_cfg;
static lemtone_solution_t   s_sol;
static lemtone_pwm_t        s_pwm;
static float                s_out_xyz[3];
static int                  s_fixed_n;  // > 0: every channel at this period (A/B against auto)

EXPORT(lt_init) void lt_init(void) {
    lemtone_init(&s_ctx, &LEMTONE_MODEL_DEFAULT);
    s_cfg = LEMTONE_PERIOD_DEFAULT;
    s_fixed_n = 0;
}

// Full pipeline (a, b: OKLab chromaticity; lm: lumens). Returns the mode.
EXPORT(lt_compute) int lt_compute(float a, float b, float lm, float f) {
    lemtone_solve(&s_ctx, a, b, lm, f, &s_sol);
    uint8_t per[LEMTONE_CH];
    lemtone_periods(&s_cfg, s_sol.duty, s_sol.lm, per);
    if (s_fixed_n > 0) {
        for (int i = 0; i < LEMTONE_CH; i++) per[i] = (uint8_t)s_fixed_n;
    }
    lemtone_quantize(&s_ctx, s_sol.duty, per, &s_pwm);
    lemtone_forward(&s_ctx, s_pwm.eff_duty, s_out_xyz);  // what the quantized command emits
    return s_sol.mode;
}

// Solve only, into the getters' state (build.py's check against the test vectors).
EXPORT(lt_solve) int lt_solve(float a, float b, float lm, float f) {
    lemtone_solve(&s_ctx, a, b, lm, f, &s_sol);
    return s_sol.mode;
}

// Solve only (no state touched): for the reachability map and timing.
EXPORT(lt_probe) int lt_probe(float a, float b, float lm, float f) {
    lemtone_solution_t s;
    lemtone_solve(&s_ctx, a, b, lm, f, &s);
    return s.mode;
}

EXPORT(lt_duty) float lt_duty(int i) { return s_sol.duty[i]; }
EXPORT(lt_desat) float lt_desat(void) { return s_sol.desat; }
EXPORT(lt_solved_lm) float lt_solved_lm(void) { return s_sol.lm; }
EXPORT(lt_evals) int lt_evals(void) { return s_sol.evals; }
EXPORT(lt_period) int lt_period(int i) { return s_pwm.period_log2[i]; }
EXPORT(lt_ticks) int lt_ticks(int i) { return s_pwm.ticks[i]; }
EXPORT(lt_cmd_duty) float lt_cmd_duty(int i) { return s_pwm.cmd_duty[i]; }
EXPORT(lt_eff_duty) float lt_eff_duty(int i) { return s_pwm.eff_duty[i]; }
EXPORT(lt_out_xyz) float lt_out_xyz(int c) { return s_out_xyz[c]; }
// Interaction factor of channel i at the last compute's light-equivalent duties
// (the page rebuilds the predicted spectrum from it).
EXPORT(lt_out_sag) float lt_out_sag(int i) { return lemtone_interaction(s_ctx.m, i, s_pwm.eff_duty); }

EXPORT(lt_fixed_period) void lt_fixed_period(int n) { s_fixed_n = n; }

EXPORT(lt_set_period_cfg)
void lt_set_period_cfg(int n_min, int n_max, int res_ticks, float lm_hi, int n_hi, float lm_lo, int n_lo) {
    s_cfg.n_min = (uint8_t)n_min;
    s_cfg.n_max = (uint8_t)n_max;
    s_cfg.res_ticks = (uint16_t)res_ticks;
    s_cfg.n_tiers = 2;
    s_cfg.tier[0].lm_above = lm_hi;
    s_cfg.tier[0].n_cap = (uint8_t)n_hi;
    s_cfg.tier[1].lm_above = lm_lo;
    s_cfg.tier[1].n_cap = (uint8_t)n_lo;
}
EXPORT(lt_cfg_n_min) int lt_cfg_n_min(void) { return s_cfg.n_min; }
EXPORT(lt_cfg_n_max) int lt_cfg_n_max(void) { return s_cfg.n_max; }
EXPORT(lt_cfg_res_ticks) int lt_cfg_res_ticks(void) { return s_cfg.res_ticks; }
EXPORT(lt_cfg_tier_lm) float lt_cfg_tier_lm(int t) { return s_cfg.tier[t].lm_above; }
EXPORT(lt_cfg_tier_n) int lt_cfg_tier_n(int t) { return s_cfg.tier[t].n_cap; }

// Colour space, same arithmetic as the solver
static float s_cs[3];
EXPORT(lt_ab_to_xyz) int lt_ab_to_xyz(float a, float b) { return lemtone_ab_to_xyz(a, b, s_cs); }
EXPORT(lt_xyz_to_ab) void lt_xyz_to_ab(float X, float Y, float Z) {
    const float v[3] = {X, Y, Z};
    lemtone_xyz_to_ab(v, s_cs);
}
EXPORT(lt_cs) float lt_cs(int c) { return s_cs[c]; }

// Model info
EXPORT(lt_forward_xyz) float lt_forward_xyz(float r, float g, float b, float ww, float cw, int c) {
    const float d[LEMTONE_CH] = {r, g, b, ww, cw};
    float X[3];
    lemtone_forward(&s_ctx, d, X);
    return X[c];
}
EXPORT(lt_lm_per_y) float lt_lm_per_y(void) { return s_ctx.m->lm_per_y; }
EXPORT(lt_duty_max) float lt_duty_max(void) { return s_ctx.m->duty_max; }
EXPORT(lt_pulse_us) float lt_pulse_us(int i) { return s_ctx.m->pulse_us[i]; }
EXPORT(lt_model_id) const char *lt_model_id(void) { return s_ctx.m->id; }
