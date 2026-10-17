// lemtone.h — Lemtone target -> per-channel PWM (duty + period) for the RGBWW bulb.
//
// Pure C with no SDK and no libc (only <stdint.h>/<stdbool.h>), so the same source builds
// for the ESP8266, the host unit tests, and WebAssembly (tools/lemtone_web).
//
// A Lemtone value is device independent except for fidelity:
//   a, b  OKLab chromaticity: OKLab a and b of the colour scaled to lightness L = 1
//         (a/L, b/L), so colour is independent of brightness. Real colours span roughly
//         a -0.83..0.79, b -1.12..0.27; anything this bulb cannot make is desaturated.
//   lm    luminous flux, lumens. 0 = off.
//   f     fidelity 0..1: how much of an unsaturated colour comes from the whites (1)
//         versus RGB (0), i.e. narrowest (0) to broadest (1) spectrum this device can
//         make. Never changes the colour or the lumens, only how the spectrum is built.
//
// The pipeline:
//   lemtone_solve()     target -> light-equivalent ("effective") duty per channel
//   lemtone_periods()   duties + lumens -> PWM period per channel (2^n us)
//   lemtone_quantize()  duties + periods -> on-time ticks per channel, corrected for the
//                       light each pulse adds beyond its tick count
//   lemtone_compute()   all three. Stateless: the same value always gives the same output.
//
// The model (lemtone_model_t) is generated from captures by
// calibration/export_lemtone_model.py; see that file for what each table means.
//
// Channel order everywhere is the wire order: r, g, b, ww, cw.

#ifndef LEMTONE_H
#define LEMTONE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEMTONE_CH        5
#define LEMTONE_MAX_KNOTS 8
#define LEMTONE_NFEAT     18

enum { LEMTONE_R = 0, LEMTONE_G, LEMTONE_B, LEMTONE_WW, LEMTONE_CW };

// Forward model: XYZ(d) = sum_i d_i * g_i(d) * T_i(d_i)
//   T_i  tristimulus per unit effective duty, piecewise linear over knot[i][]
//        (held constant outside the knots)
//   g_i  1 + feats_i(d) . theta[i]: channel interaction (supply sag). feats_i are the
//        other four duties o_j, then o_j * d_i, then every o_a * o_b with a <= b.
typedef struct {
    const char *id;
    uint8_t     n_knots;
    float       knot[LEMTONE_CH][LEMTONE_MAX_KNOTS];      // effective duty, ascending
    float       xyz[LEMTONE_CH][LEMTONE_MAX_KNOTS][3];    // XYZ per unit effective duty
    float       theta[LEMTONE_CH][LEMTONE_NFEAT];
    float       pulse_us[LEMTONE_CH];  // light per pulse beyond its tick count, in us
    float       duty_max;              // effective-duty cap (PWM_MAX_POWER)
    float       lm_per_y;              // lumens per unit of model Y
} lemtone_model_t;

extern const lemtone_model_t LEMTONE_MODEL_DEFAULT;

// Precomputed from the model by lemtone_init(); read-only afterwards.
typedef struct {
    const lemtone_model_t *m;
    float prim[LEMTONE_CH][3];  // XYZ per unit duty at the reference duty (first round)
    float cw_ab[2], ww_ab[2];   // ends of the white line, OKLab chromaticity
} lemtone_ctx_t;

typedef enum {
    LEMTONE_OFF = 0,    // lm <= 0 (or not a number): all channels off
    LEMTONE_EXACT,      // target reached
    LEMTONE_DESAT,      // lumens kept by moving the colour toward the white line (in OKLab)
    LEMTONE_CLAMP,      // even white cannot reach lm: brightest white instead
} lemtone_mode_t;

typedef struct {
    float    duty[LEMTONE_CH];  // effective duty, 0..duty_max
    uint8_t  mode;              // lemtone_mode_t
    float    desat;             // saturation given up, 0 (none) .. 1 (all)
    float    lm;                // lumens the duties deliver per the model
    uint16_t evals;             // forward-model evaluations spent
} lemtone_solution_t;

// Period choice. A channel's period grows from n_min until its on-time reaches
// res_ticks or it hits the brightness cap: the first tier whose lm_above the total
// lumens exceed sets the cap (tiers sorted by descending lm_above), else n_max.
// Dim light is where flicker is hardest to see, so it gets the long periods.
#define LEMTONE_MAX_TIERS 4
typedef struct {
    uint8_t  n_min;
    uint8_t  n_max;
    uint16_t res_ticks;
    uint8_t  n_tiers;
    struct {
        float   lm_above;
        uint8_t n_cap;
    } tier[LEMTONE_MAX_TIERS];
} lemtone_period_cfg_t;

// n 10..14; 200 ticks; > 180 lm -> 2^10, > 60 lm -> 2^12, else 2^14.
extern const lemtone_period_cfg_t LEMTONE_PERIOD_DEFAULT;

typedef struct {
    uint8_t  period_log2[LEMTONE_CH];
    uint16_t ticks[LEMTONE_CH];     // on-time per period, in 1 us ticks
    float    cmd_duty[LEMTONE_CH];  // ticks / 2^n: the duty pwm_compile() turns back into `ticks`
    float    eff_duty[LEMTONE_CH];  // light-equivalent duty of `ticks`
} lemtone_pwm_t;

void lemtone_init(lemtone_ctx_t *ctx, const lemtone_model_t *m);

// Model output for effective duties d.
void lemtone_forward(const lemtone_ctx_t *ctx, const float d[LEMTONE_CH], float xyz[3]);

// Interaction factor g_i(d) of channel i (1 = no sag), as used by lemtone_forward.
float lemtone_interaction(const lemtone_model_t *m, int i, const float d[LEMTONE_CH]);

void lemtone_solve(const lemtone_ctx_t *ctx, float a, float b, float lm, float fidelity,
                   lemtone_solution_t *out);

// Channels too dim for one tick even at n_max (effectively off) get n_min.
void lemtone_periods(const lemtone_period_cfg_t *cfg, const float duty[LEMTONE_CH], float lm,
                     uint8_t out[LEMTONE_CH]);

void lemtone_quantize(const lemtone_ctx_t *ctx, const float duty[LEMTONE_CH],
                      const uint8_t period_log2[LEMTONE_CH], lemtone_pwm_t *out);

// solve -> periods -> quantize.
void lemtone_compute(const lemtone_ctx_t *ctx, const lemtone_period_cfg_t *cfg,
                     float a, float b, float lm, float fidelity,
                     lemtone_solution_t *sol, lemtone_pwm_t *pwm);

// OKLab chromaticity (a/L, b/L) -> CIE XYZ with Y = 1. False if the colour has no
// positive luminance (far outside the real colours).
bool lemtone_ab_to_xyz(float a, float b, float xyz[3]);

// CIE XYZ (Y > 0) -> OKLab chromaticity (a/L, b/L).
void lemtone_xyz_to_ab(const float xyz[3], float ab[2]);

#ifdef __cplusplus
}
#endif

#endif  // LEMTONE_H
