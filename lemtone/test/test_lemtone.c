// test_lemtone.c — host tests for lemtone.c against the Python reference
// (calibration/lemtone_ref.py writes lemtone_vectors.h from the same model).
//
// Build (any host C compiler), from bulb-firmware/lemtone:
//   cl /nologo /W3 /I. test\test_lemtone.c lemtone.c lemtone_model_default.c
//   cc -I. test/test_lemtone.c lemtone.c lemtone_model_default.c -o test_lemtone

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "lemtone.h"
// Vectors for the model being linked: build.py passes -DLEMTONE_VECTORS for models other than
// the default (bulb) one. Each header carries the model's rated output; the bulb's predates
// that and is the fallback below.
#ifdef LEMTONE_VECTORS
#include LEMTONE_VECTORS
#else
#include "lemtone_vectors.h"
#endif
#ifndef RATED_LM
#define RATED_LM 1000.0f
#define RATED_DUTY {0, 0, 0, ctx.m->duty_max, ctx.m->duty_max}
#endif

#define N_OF(a) ((int)(sizeof(a) / sizeof((a)[0])))

static int s_checks, s_fails;
#define CHECK(cond, ...)                                    \
    do {                                                    \
        s_checks++;                                         \
        if (!(cond)) {                                      \
            s_fails++;                                      \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);     \
            printf(__VA_ARGS__);                            \
            printf("\n");                                   \
        }                                                   \
    } while (0)

static lemtone_ctx_t ctx;

static void test_forward(void) {
    float worst = 0;
    for (int k = 0; k < N_OF(FWD_VECS); k++) {
        float X[3];
        lemtone_forward(&ctx, FWD_VECS[k].d, X);
        for (int c = 0; c < 3; c++) {
            const float ref = FWD_VECS[k].xyz[c];
            const float err = fabsf(X[c] - ref) / (fabsf(ref) + 1.0f);
            if (err > worst) worst = err;
        }
    }
    CHECK(worst < 1e-4f, "forward model differs from reference by %.2e (relative)", worst);
    printf("forward: %d vectors, worst relative difference %.2e\n", N_OF(FWD_VECS), worst);
}

static void test_solve_vs_reference(void) {
    int mode_diff = 0, n_exact = 0;
    float worst_duty = 0, worst_resid = 0;
    int tick_diff = 0, period_diff = 0;
    for (int k = 0; k < N_OF(SOLVE_VECS); k++) {
        const solve_vec_t *v = &SOLVE_VECS[k];
        lemtone_solution_t s;
        lemtone_solve(&ctx, v->a, v->b, v->lm, v->f, &s);
        if (s.mode != v->mode) {
            mode_diff++;
            printf("  mode differs at #%d (a %.4f b %.4f lm %.4f f %.1f): C %d ref %d\n",
                   k, v->a, v->b, v->lm, v->f, s.mode, v->mode);
            continue;
        }
        float dd = 0;
        for (int i = 0; i < 5; i++) dd = fmaxf(dd, fabsf(s.duty[i] - v->duty[i]));
        // Bisection can settle one step apart in float32; only EXACT has to match tightly.
        if (s.mode == LEMTONE_EXACT) {
            n_exact++;
            if (dd > worst_duty) worst_duty = dd;
            float X[3];
            lemtone_forward(&ctx, s.duty, X);
            const float Y = v->lm / ctx.m->lm_per_y;
            float tgt[3];
            lemtone_ab_to_xyz(v->a, v->b, tgt);
            for (int c = 0; c < 3; c++) worst_resid = fmaxf(worst_resid, fabsf(X[c] - tgt[c] * Y) / Y);

            uint8_t per[5];
            lemtone_pwm_t pwm;
            lemtone_periods(&LEMTONE_PERIOD_DEFAULT, s.duty, s.lm, per);
            lemtone_quantize(&ctx, s.duty, per, &pwm);
            for (int i = 0; i < 5; i++) {
                if (per[i] != v->period[i]) {
                    period_diff++;
                    printf("  period differs at #%d ch %d: C %d ref %d (duty %.6f, %.2f ticks at 2^%d, %.2f lm)\n",
                           k, i, per[i], v->period[i], s.duty[i], s.duty[i] * (1 << v->period[i]),
                           v->period[i], s.lm);
                }
                if (abs((int)pwm.ticks[i] - (int)v->ticks[i]) > 1) {
                    tick_diff++;
                    printf("  ticks differ at #%d ch %d: C %u ref %u (duty %.6f)\n", k, i, pwm.ticks[i], v->ticks[i], s.duty[i]);
                }
            }
        } else {
            CHECK(dd < 0.02f || fabsf(s.desat - v->desat) < 0.01f,
                  "#%d mode %d: duties differ by %.4f, desat C %.3f ref %.3f", k, s.mode, dd, s.desat, v->desat);
        }
    }
    printf("solve: %d vectors, %d mode mismatches, %d exact: worst duty diff %.2e, worst residual %.2e of Y\n",
           N_OF(SOLVE_VECS), mode_diff, n_exact, worst_duty, worst_resid);
    CHECK(mode_diff <= N_OF(SOLVE_VECS) / 100, "%d mode mismatches (allowed: 1%%, boundary cases)", mode_diff);
    CHECK(worst_duty < 2e-3f, "exact solve duties differ from reference by %.2e", worst_duty);
    // Driver whites sit on a 1/256 DIM grid and R, G, B make up the rest; at the gamut edge
    // they have no room left, so a few targets miss by up to ~0.6%.
    const float resid_tol = ctx.m->white_grid_log2 ? 1e-2f : 2.5e-3f;
    CHECK(worst_resid < resid_tol, "exact solve misses its target by %.2e of Y", worst_resid);
    CHECK(period_diff == 0, "%d periods differ from reference", period_diff);
    CHECK(tick_diff == 0, "%d tick counts differ from reference by more than 1", tick_diff);
}

// Chromaticity and lumens of the model's own output at duties d.
static void light_of(const float d[5], float ab[2], float *lm) {
    float X[3];
    lemtone_forward(&ctx, d, X);
    lemtone_xyz_to_ab(X, ab);
    *lm = X[1] * ctx.m->lm_per_y;
}

static void test_colour_space(void) {
    // OKLab round trip, and D65 sits at a = b = 0 (to the precision of the published matrices).
    float worst = 0;
    const float pts[][2] = {{0, 0}, {0.2f, 0.1f}, {-0.2f, 0.15f}, {0.05f, -0.3f}, {0.15f, -0.25f}};
    for (int k = 0; k < N_OF(pts); k++) {
        float X[3], ab[2];
        CHECK(lemtone_ab_to_xyz(pts[k][0], pts[k][1], X), "ab %.2f %.2f has no luminance", pts[k][0], pts[k][1]);
        CHECK(fabsf(X[1] - 1.0f) < 1e-6f, "ab_to_xyz Y = %f, want 1", X[1]);
        lemtone_xyz_to_ab(X, ab);
        worst = fmaxf(worst, fmaxf(fabsf(ab[0] - pts[k][0]), fabsf(ab[1] - pts[k][1])));
    }
    CHECK(worst < 2e-6f, "OKLab round trip off by %.2e", worst);
    float ab[2];
    const float d65[3] = {0.95047f, 1.0f, 1.08883f};
    lemtone_xyz_to_ab(d65, ab);
    CHECK(fabsf(ab[0]) < 1e-3f && fabsf(ab[1]) < 1e-3f, "D65 at a %.5f b %.5f, want ~0", ab[0], ab[1]);
}

static void test_off_and_edges(void) {
    lemtone_solution_t s;
    lemtone_solve(&ctx, 0.0f, 0.0f, 0.0f, 0.5f, &s);
    CHECK(s.mode == LEMTONE_OFF, "0 lm should be OFF, got %d", s.mode);
    for (int i = 0; i < 5; i++) CHECK(s.duty[i] == 0.0f, "OFF duty %d = %f", i, s.duty[i]);
    lemtone_solve(&ctx, 0.0f, 0.0f, -3.0f, 0.5f, &s);
    CHECK(s.mode == LEMTONE_OFF, "negative lm should be OFF, got %d", s.mode);
    lemtone_solve(&ctx, 0.0f, 0.0f, 1e6f, 1.0f, &s);
    CHECK(s.mode == LEMTONE_CLAMP && s.lm > 0.5f * RATED_LM && s.lm < 1.1f * RATED_LM,
          "1e6 lm should clamp, got mode %d %.1f lm", s.mode, s.lm);
    for (int i = 0; i < 5; i++) CHECK(s.duty[i] <= ctx.m->duty_max, "clamped duty %d = %f over cap", i, s.duty[i]);

    // Imaginary chromaticities desaturate like any unreachable colour instead of switching off.
    const float imag[][2] = {{0.79f, -1.12f}, {-0.83f, 0.27f}, {0.0f, -1.0f}, {-0.6f, -0.6f}};
    for (int k = 0; k < N_OF(imag); k++) {
        lemtone_solve(&ctx, imag[k][0], imag[k][1], 20.0f, 0.5f, &s);
        CHECK(s.mode == LEMTONE_DESAT && s.lm == 20.0f, "imaginary ab %.2f %.2f: mode %d %.2f lm, want DESAT 20",
              imag[k][0], imag[k][1], s.mode, s.lm);
    }

    // The rated output holds by construction of lm_per_y.
    float ab[2], lm;
    const float rated[5] = RATED_DUTY;
    light_of(rated, ab, &lm);
    CHECK(fabsf(lm - RATED_LM) < 0.05f, "rated duties give %.2f lm, want %.0f", lm, RATED_LM);

    // Asking for exactly WW at the cap at full fidelity gives WW alone back (fidelity
    // must hold on the full model, not only on its linearisation).
    const float ww_cap[5] = {0, 0, 0, ctx.m->duty_max, 0};
    light_of(ww_cap, ab, &lm);
    lemtone_solve(&ctx, ab[0], ab[1], lm, 1.0f, &s);
    CHECK(s.mode == LEMTONE_EXACT, "WW at cap: mode %d", s.mode);
    CHECK(fabsf(s.duty[LEMTONE_WW] - ctx.m->duty_max) < 0.01f && s.duty[0] < 0.01f && s.duty[1] < 0.01f &&
              s.duty[2] < 0.01f && s.duty[4] < 0.01f,
          "WW at cap, F1: duties %.3f %.3f %.3f %.3f %.3f, want WW %.2f alone", s.duty[0], s.duty[1], s.duty[2],
          s.duty[3], s.duty[4], ctx.m->duty_max);
    // Same for CW at half power (on the driver's DIM grid where there is one).
    const float cw_d = ctx.m->white_grid_log2
                           ? lroundf(0.4f * (1 << ctx.m->white_grid_log2)) / (float)(1 << ctx.m->white_grid_log2)
                           : 0.4f;
    const float cw_half[5] = {0, 0, 0, 0, cw_d};
    light_of(cw_half, ab, &lm);
    lemtone_solve(&ctx, ab[0], ab[1], lm, 1.0f, &s);
    CHECK(fabsf(s.duty[LEMTONE_CW] - cw_d) < 0.01f && s.duty[0] < 0.01f && s.duty[1] < 0.01f && s.duty[2] < 0.01f &&
              s.duty[3] < 0.01f,
          "CW %.3f, F1: duties %.3f %.3f %.3f %.3f %.3f, want CW alone", cw_d, s.duty[0], s.duty[1], s.duty[2],
          s.duty[3], s.duty[4]);
}

static void test_periods(void) {
    const lemtone_period_cfg_t *cfg = &LEMTONE_PERIOD_DEFAULT;
    uint8_t per[5];
    // Bright: every lit channel stays at n_min whatever its share.
    const float bright[5] = {0.02f, 0.0f, 0.0f, 0.6f, 0.0f};
    lemtone_periods(cfg, bright, 600.0f, per);
    CHECK(per[0] == 10 && per[3] == 10, "bright scene periods %d/%d, want 10/10", per[0], per[3]);
    // Mid brightness: dim channel capped at 2^12.
    lemtone_periods(cfg, bright, 120.0f, per);
    CHECK(per[0] == 12 && per[3] == 10, "mid scene periods %d/%d, want 12/10", per[0], per[3]);
    // Red night light: dim and dark -> longest period, ~164 ticks of resolution.
    const float night[5] = {0.01f, 0, 0, 0, 0};
    lemtone_periods(cfg, night, 0.5f, per);
    CHECK(per[0] == 14, "night light period %d, want 14", per[0]);
    // Resolution reached early: 0.1 duty has 205 ticks at 2^11.
    const float tenth[5] = {0.1f, 0, 0, 0, 0};
    lemtone_periods(cfg, tenth, 5.0f, per);
    CHECK(per[0] == 11, "0.1 duty period %d, want 11", per[0]);
    // Off channels report n_min.
    CHECK(per[1] == 10, "off channel period %d, want 10", per[1]);
    // Tier edges: strictly above lm_above.
    lemtone_periods(cfg, night, 60.0f, per);
    CHECK(per[0] == 14, "at exactly 60 lm period %d, want 14", per[0]);
    lemtone_periods(cfg, night, 60.01f, per);
    CHECK(per[0] == 12, "just above 60 lm period %d, want 12", per[0]);
    lemtone_periods(cfg, night, 180.01f, per);
    CHECK(per[0] == 10, "just above 180 lm period %d, want 10", per[0]);
}

static void test_quantize(void) {
    const float duty[5] = {0.01f, 0.3f, 0.0001f, 0.8f, 0.05f};
    const uint8_t per[5] = {14, 10, 10, 10, 12};
    lemtone_pwm_t q;
    lemtone_quantize(&ctx, duty, per, &q);
    const int n_direct = ctx.m->white_grid_log2 ? 3 : 5;  // driver whites: checked below
    for (int i = 0; i < n_direct; i++) {
        const float L = (float)(1u << per[i]);
        // what pwm_compile does with the command: lroundf(cmd * 2^n)
        CHECK(lroundf(q.cmd_duty[i] * L) == q.ticks[i], "ch %d cmd %.8f does not round-trip to %u ticks",
              i, q.cmd_duty[i], q.ticks[i]);
        CHECK(q.cmd_duty[i] <= ctx.m->duty_max, "ch %d command %.5f above the cap", i, q.cmd_duty[i]);
        if (q.ticks[i]) {
            CHECK(fabsf(q.eff_duty[i] - duty[i]) <= 0.5f / L + 1e-7f,
                  "ch %d effective %.6f vs asked %.6f: more than half a tick", i, q.eff_duty[i], duty[i]);
        }
    }
    if (ctx.m->white_grid_log2) {
        // what pwm_mix + pwm_compile do with the white pair: DIM = ww + cw and CCT = ww / DIM,
        // each lroundf(x * 2^n) at the driver's fixed period
        const float L = (float)(1u << ctx.m->white_grid_log2);
        const float ww = q.cmd_duty[LEMTONE_WW], cw = q.cmd_duty[LEMTONE_CW];
        CHECK(lroundf((ww + cw) * L) == q.ticks[LEMTONE_CW], "DIM %.6f does not round-trip to %u ticks",
              ww + cw, q.ticks[LEMTONE_CW]);
        CHECK(lroundf(ww / (ww + cw) * L) == q.ticks[LEMTONE_WW], "CCT %.6f does not round-trip to %u ticks",
              ww / (ww + cw), q.ticks[LEMTONE_WW]);
        CHECK(fabsf(ww + cw - (duty[3] + duty[4])) <= 0.5f / L + 1e-6f, "DIM %.5f vs asked %.5f", ww + cw,
              duty[3] + duty[4]);
    }
    if (lemtone_pulse_us(ctx.m, 2, per[2]) >= 0.0f) {
        CHECK(q.ticks[2] == 0, "a duty below one pulse's light should be off, got %u ticks", q.ticks[2]);
    } else {  // pulses that lose light: the first ~|pulse| ticks are spent before any light
        CHECK(q.ticks[2] <= (uint16_t)(1.5f - lemtone_pulse_us(ctx.m, 2, per[2])),
              "a duty of 0.1 tick of light took %u ticks", q.ticks[2]);
    }
    // The pulse correction: 0.01 at 2^14 is 163.84 ticks of light, minus the pulse's own light.
    CHECK(q.ticks[0] == (uint16_t)lroundf(0.01f * 16384 - lemtone_pulse_us(ctx.m, 0, 14)), "red ticks %u",
          q.ticks[0]);
}

static void test_compute_is_stateless(void) {
    // A slow log fade down then up: periods follow the lumens only, so the way up retraces
    // the way down exactly, and each channel's period changes monotonically each way.
    float ab[2], lm0;
    const float red[5] = {0.4f, 0.02f, 0, 0, 0};
    light_of(red, ab, &lm0);
    lemtone_solution_t s;
    lemtone_pwm_t q;
    uint8_t down[201][5];
    int mismatch = 0, dir_changes = 0, last_dir = 0, last = -1;
    for (int k = 0; k <= 400; k++) {
        const float t = k <= 200 ? k / 200.0f : (400 - k) / 200.0f;  // 0 -> 1 -> 0
        const float lm = 300.0f * powf(0.0005f, t);                    // 300 .. 0.15 lm
        lemtone_compute(&ctx, &LEMTONE_PERIOD_DEFAULT, ab[0], ab[1], lm, 0.0f, &s, &q);
        const int kk = k <= 200 ? k : 400 - k;
        for (int i = 0; i < 5; i++) {
            if (k <= 200) down[kk][i] = q.period_log2[i];
            else if (down[kk][i] != q.period_log2[i]) mismatch++;
        }
        if (last >= 0 && q.period_log2[0] != last) {
            const int dir = q.period_log2[0] > last ? 1 : -1;
            if (last_dir && dir != last_dir) dir_changes++;
            last_dir = dir;
        }
        last = q.period_log2[0];
    }
    printf("fade: %d period mismatches between the way down and up, %d direction reversals\n", mismatch,
           dir_changes);
    CHECK(mismatch == 0, "%d periods differ between equal lumens on the way down and up", mismatch);
    CHECK(dir_changes == 1, "red period reversed direction %d times over one down/up fade", dir_changes);
}

int main(void) {
    lemtone_init(&ctx, &LEMTONE_MODEL_DEFAULT);
    printf("model %s\n", ctx.m->id);
    test_forward();
    test_colour_space();
    test_solve_vs_reference();
    test_off_and_edges();
    test_periods();
    test_quantize();
    test_compute_is_stateless();
    printf("%d/%d checks passed\n", s_checks - s_fails, s_checks);
    return s_fails ? 1 : 0;
}
