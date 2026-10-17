// lemtone.c — Lemtone target -> per-channel duty + PWM period (see lemtone.h).
//
// The solve:
//   1. Closed form on a linear model XYZ = sum_i d_i P_i, as lemtone.html does: the
//      white level `we` is split CW/WW by where the target projects onto the white line,
//      r,g,b come from the 3x3 primary inverse, and the range of `we` that keeps every
//      duty in [0, duty_max] is found directly; fidelity picks a point in that range.
//   2. Frozen coefficients: P_i starts as each channel's output per unit duty at
//      REF_DUTY, then is re-evaluated at the duties just found (interaction and the
//      duty-dependent colour included) and step 1 runs again, until the full model
//      lands on the target. Because step 1 is redone with the true local P_i, the
//      fidelity choice holds for the real model too (WW alone stays WW alone).
//   3. Infeasible target: bisect the saturation along the OKLab line toward the target's
//      point on the white line, at the same lumens. If even that point cannot reach the
//      lumens, bisect the lumens there.
//
// Targets arrive as OKLab chromaticity + lumens and are turned into XYZ once; the solve
// itself runs in XYZ, where the model is (nearly) linear.

#include "lemtone.h"

#define REF_DUTY     0.4f   // where the closed form's primaries are taken
#define PHASE1_ROUNDS 6     // rounds without the interaction term
#define FIX_ROUNDS   12     // rounds with it
#define FIX_TOL      2e-3f  // converged when every XYZ error < FIX_TOL * Y
#define BISECT_STEPS 10
#define EPS          1e-6f

const lemtone_period_cfg_t LEMTONE_PERIOD_DEFAULT = {
    .n_min = 10,
    .n_max = 14,
    .res_ticks = 200,
    .n_tiers = 2,
    .tier = {{180.0f, 10}, {60.0f, 12}},
};

// OKLab (Ottosson 2020): XYZ -> LMS (M1), cube root, -> Lab (M2), and back.
static const float OK_M1[3][3] = {{0.8189330101f, 0.3618667424f, -0.1288597137f},
                                  {0.0329845436f, 0.9293118715f, 0.0361456387f},
                                  {0.0482003018f, 0.2643662691f, 0.6338517070f}};
static const float OK_M2[3][3] = {{0.2104542553f, 0.7936177850f, -0.0040720468f},
                                  {1.9779984951f, -2.4285922050f, 0.4505937099f},
                                  {0.0259040371f, 0.7827717662f, -0.8086757660f}};
static const float OK_M2_INV[3][3] = {{1.0f, 0.3963377774f, 0.2158037573f},
                                      {1.0f, -0.1055613458f, -0.0638541728f},
                                      {1.0f, -0.0894841775f, -1.2914855480f}};
static const float OK_M1_INV[3][3] = {{1.2270138511f, -0.5577999807f, 0.2812561490f},
                                      {-0.0405801784f, 1.1122568696f, -0.0716766787f},
                                      {-0.0763812845f, -0.4214819784f, 1.5861632204f}};

static inline float absf(float v) { return v < 0.0f ? -v : v; }
static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- forward model ----------------------------------------------------------

static void unit_xyz(const lemtone_model_t *m, int i, float d, float out[3]) {
    const float *k = m->knot[i];
    const int    n = m->n_knots;
    if (d <= k[0] || n == 1) {
        for (int c = 0; c < 3; c++) out[c] = m->xyz[i][0][c];
        return;
    }
    if (d >= k[n - 1]) {
        for (int c = 0; c < 3; c++) out[c] = m->xyz[i][n - 1][c];
        return;
    }
    int j = 1;
    while (d > k[j]) j++;
    const float t = (d - k[j - 1]) / (k[j] - k[j - 1]);
    for (int c = 0; c < 3; c++) {
        out[c] = m->xyz[i][j - 1][c] + t * (m->xyz[i][j][c] - m->xyz[i][j - 1][c]);
    }
}

// Interaction factor g_i. Feature order matches calibration/analyze_mix.quad_feats.
float lemtone_interaction(const lemtone_model_t *m, int i, const float d[LEMTONE_CH]) {
    float o[LEMTONE_CH - 1];
    int   n = 0;
    for (int j = 0; j < LEMTONE_CH; j++) {
        if (j != i) o[n++] = d[j];
    }
    const float *th = m->theta[i];
    float g = 1.0f;
    int   f = 0;
    for (int a = 0; a < 4; a++) g += th[f++] * o[a];
    for (int a = 0; a < 4; a++) g += th[f++] * o[a] * d[i];
    for (int a = 0; a < 4; a++) {
        for (int b = a; b < 4; b++) g += th[f++] * o[a] * o[b];
    }
    return g;
}

void lemtone_forward(const lemtone_ctx_t *ctx, const float d[LEMTONE_CH], float xyz[3]) {
    xyz[0] = xyz[1] = xyz[2] = 0.0f;
    for (int i = 0; i < LEMTONE_CH; i++) {
        if (d[i] <= 0.0f) continue;
        float u[3];
        unit_xyz(ctx->m, i, d[i], u);
        const float s = d[i] * lemtone_interaction(ctx->m, i, d);
        for (int c = 0; c < 3; c++) xyz[c] += s * u[c];
    }
}

// ---- colour space ---------------------------------------------------------------

static void mul3(const float m[3][3], const float v[3], float out[3]) {
    for (int r = 0; r < 3; r++) out[r] = m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2];
}

// Cube root without libm: bit-level first guess, then Newton (exact to float precision).
static float cbrtf_(float v) {
    if (v == 0.0f) return 0.0f;
    const bool neg = v < 0.0f;
    if (neg) v = -v;
    union { float f; uint32_t u; } g = {v};
    g.u = g.u / 3u + 709921077u;
    float y = g.f;
    for (int k = 0; k < 4; k++) y -= (y * y * y - v) / (3.0f * y * y);
    return neg ? -y : y;
}

bool lemtone_ab_to_xyz(float a, float b, float xyz[3]) {
    const float lab[3] = {1.0f, a, b};
    float l[3];
    mul3(OK_M2_INV, lab, l);
    for (int c = 0; c < 3; c++) l[c] = l[c] * l[c] * l[c];
    mul3(OK_M1_INV, l, xyz);
    if (!(xyz[1] > 1e-6f)) return false;
    const float k = 1.0f / xyz[1];
    for (int c = 0; c < 3; c++) xyz[c] *= k;
    return true;
}

void lemtone_xyz_to_ab(const float xyz[3], float ab[2]) {
    float l[3], lab[3];
    mul3(OK_M1, xyz, l);
    for (int c = 0; c < 3; c++) l[c] = cbrtf_(l[c]);
    mul3(OK_M2, l, lab);
    ab[0] = lab[1] / lab[0];
    ab[1] = lab[2] / lab[0];
}

// ---- init ---------------------------------------------------------------------

static void xy_of(const float X[3], float out[2]) {
    const float s = X[0] + X[1] + X[2];
    out[0] = s > 0.0f ? X[0] / s : 0.0f;
    out[1] = s > 0.0f ? X[1] / s : 0.0f;
}

void lemtone_init(lemtone_ctx_t *ctx, const lemtone_model_t *m) {
    ctx->m = m;
    for (int i = 0; i < LEMTONE_CH; i++) unit_xyz(m, i, REF_DUTY, ctx->prim[i]);
    lemtone_xyz_to_ab(ctx->prim[LEMTONE_CW], ctx->cw_ab);
    lemtone_xyz_to_ab(ctx->prim[LEMTONE_WW], ctx->ww_ab);
}

// ---- solve --------------------------------------------------------------------

// Where p projects onto the line cw -> ww: 0 = all CW, 1 = all WW.
static float beta_on(const float cw[2], const float ww[2], const float p[2]) {
    const float dx = ww[0] - cw[0], dy = ww[1] - cw[1];
    const float t = ((p[0] - cw[0]) * dx + (p[1] - cw[1]) * dy) / (dx * dx + dy * dy);
    return clampf(t, 0.0f, 1.0f);
}


// Inverse of the matrix whose columns are the r, g, b primaries. False if singular.
static bool inv_rgb(const float P[LEMTONE_CH][3], float out[3][3]) {
    const float a = P[0][0], b = P[1][0], c = P[2][0];
    const float d = P[0][1], e = P[1][1], f = P[2][1];
    const float g = P[0][2], h = P[1][2], k = P[2][2];
    const float A = e * k - f * h, B = -(d * k - f * g), C = d * h - e * g;
    const float det = a * A + b * B + c * C;
    if (absf(det) < 1e-12f) return false;
    out[0][0] = A / det;
    out[0][1] = -(b * k - c * h) / det;
    out[0][2] = (b * f - c * e) / det;
    out[1][0] = B / det;
    out[1][1] = (a * k - c * g) / det;
    out[1][2] = -(a * f - c * d) / det;
    out[2][0] = C / det;
    out[2][1] = -(a * h - b * g) / det;
    out[2][2] = (a * e - b * d) / det;
    return true;
}

// Closed form for a linear model XYZ = sum_i d_i P[i]: duties giving `tgt` (chromaticity
// p), whites split by where p projects onto THIS model's CW-WW line, white level at
// fraction f of its feasible range. False if no white level keeps every duty in
// [0, duty_max].
static bool closed_form(const lemtone_ctx_t *ctx, const float P[LEMTONE_CH][3], const float tgt[3],
                        const float p[2], float f, float d[LEMTONE_CH]) {
    const float dmax = ctx->m->duty_max;
    float ainv[3][3], wmix[3], a[3], b[3], cw[2], ww[2];
    if (!inv_rgb(P, ainv)) return false;
    xy_of(P[LEMTONE_CW], cw);
    xy_of(P[LEMTONE_WW], ww);
    const float beta = beta_on(cw, ww, p);
    for (int c = 0; c < 3; c++) {
        wmix[c] = (1.0f - beta) * P[LEMTONE_CW][c] + beta * P[LEMTONE_WW][c];
    }
    mul3(ainv, tgt, a);   // rgb(we) = a - we * b
    mul3(ainv, wmix, b);

    float lo = 0.0f, hi = dmax / maxf(maxf(beta, 1.0f - beta), EPS);
    const float tol = EPS * dmax;
    for (int k = 0; k < 3; k++) {
        if (absf(b[k]) < 1e-12f) {
            if (a[k] < -tol || a[k] > dmax + tol) return false;
            continue;
        }
        float l = (a[k] - dmax) / b[k], h = a[k] / b[k];
        if (b[k] < 0.0f) {
            const float t = l;
            l = h;
            h = t;
        }
        lo = maxf(lo, l);
        hi = minf(hi, h);
    }
    if (hi < lo - tol) return false;

    const float we = lo + f * maxf(0.0f, hi - lo);
    for (int k = 0; k < 3; k++) d[k] = clampf(a[k] - we * b[k], 0.0f, dmax);
    d[LEMTONE_WW] = clampf(we * beta, 0.0f, dmax);
    d[LEMTONE_CW] = clampf(we * (1.0f - beta), 0.0f, dmax);
    return true;
}

// Per-unit-duty output of every channel at duties d, interaction included:
// P[i] = g_i(d) * T_i(d_i), so that XYZ(d) = sum_i d_i P[i] exactly.
static void local_primaries(const lemtone_ctx_t *ctx, const float d[LEMTONE_CH], bool with_sag,
                            float P[LEMTONE_CH][3]) {
    for (int i = 0; i < LEMTONE_CH; i++) {
        unit_xyz(ctx->m, i, d[i], P[i]);
        const float g = with_sag ? lemtone_interaction(ctx->m, i, d) : 1.0f;
        for (int c = 0; c < 3; c++) P[i][c] *= g;
    }
}

// Frozen-coefficient rounds: closed form with P, re-evaluate P at the new duties,
// until sum_i d_i P_i(d) lands on tgt. True when converged; d holds the last duties.
static bool rounds(const lemtone_ctx_t *ctx, const float tgt[3], const float p[2], float Y, float f,
                   bool with_sag, int max_rounds, float P[LEMTONE_CH][3], float d[LEMTONE_CH],
                   bool *feasible, uint16_t *evals) {
    *feasible = true;
    for (int it = 0; it < max_rounds; it++) {
        if (!closed_form(ctx, P, tgt, p, f, d)) {
            *feasible = false;
            return false;
        }
        local_primaries(ctx, d, with_sag, P);
        (*evals)++;
        float worst = 0.0f;
        for (int c = 0; c < 3; c++) {
            float out = 0.0f;
            for (int i = 0; i < LEMTONE_CH; i++) out += d[i] * P[i][c];
            worst = maxf(worst, absf(tgt[c] - out));
        }
        if (worst < FIX_TOL * Y) return true;
    }
    return false;
}

// XYZ target tgt (luminance tgt[1]), in two phases of frozen-coefficient rounds:
//   1. without the interaction term: settles the white/RGB split where fidelity puts
//      it, following only each channel's duty-dependent colour.
//   2. with it, starting from phase 1: corrects for the supply sag.
// Going straight to phase 2 can lock onto a worse fixed point: a little RGB in the
// first round sags the whites, which then cannot reach the target alone, which keeps
// the RGB on. At convergence the closed form (and so the fidelity choice) holds for
// the full model, not just its linearisation at REF_DUTY.
static bool attempt(const lemtone_ctx_t *ctx, const float tgt[3], float f, float d[LEMTONE_CH],
                    uint16_t *evals) {
    const float Y = tgt[1];
    if (!(Y > 0.0f) || tgt[0] < 0.0f || tgt[2] < 0.0f) return false;  // not a real colour
    float p[2];
    xy_of(tgt, p);
    float P[LEMTONE_CH][3];
    for (int i = 0; i < LEMTONE_CH; i++) {
        for (int c = 0; c < 3; c++) P[i][c] = ctx->prim[i][c];
    }
    bool feasible;
    if (!rounds(ctx, tgt, p, Y, f, false, PHASE1_ROUNDS, P, d, &feasible, evals) && !feasible) {
        // infeasible without sag: start phase 2 from scratch rather than give up
        for (int i = 0; i < LEMTONE_CH; i++) {
            for (int c = 0; c < 3; c++) P[i][c] = ctx->prim[i][c];
        }
    } else {
        local_primaries(ctx, d, true, P);
    }
    return rounds(ctx, tgt, p, Y, f, true, FIX_ROUNDS, P, d, &feasible, evals);
}

static void copy5(float dst[LEMTONE_CH], const float src[LEMTONE_CH]) {
    for (int i = 0; i < LEMTONE_CH; i++) dst[i] = src[i];
}

// XYZ of OKLab chromaticity ab at luminance Y. False if ab has no positive luminance.
static bool target_xyz(const float ab[2], float Y, float out[3]) {
    if (!lemtone_ab_to_xyz(ab[0], ab[1], out)) return false;
    for (int c = 0; c < 3; c++) out[c] *= Y;
    return true;
}

static bool attempt_ab(const lemtone_ctx_t *ctx, const float ab[2], float Y, float f,
                       float d[LEMTONE_CH], uint16_t *evals) {
    float tgt[3];
    return target_xyz(ab, Y, tgt) && attempt(ctx, tgt, f, d, evals);
}

void lemtone_solve(const lemtone_ctx_t *ctx, float a, float b, float lm, float fidelity,
                   lemtone_solution_t *out) {
    out->evals = 0;
    out->desat = 0.0f;
    for (int i = 0; i < LEMTONE_CH; i++) out->duty[i] = 0.0f;
    if (!(lm > 1e-9f) || a != a || b != b) {
        out->mode = LEMTONE_OFF;
        out->lm = 0.0f;
        return;
    }
    const float f = clampf(fidelity, 0.0f, 1.0f);
    const float Y = lm / ctx->m->lm_per_y;
    const float p[2] = {a, b};
    float d[LEMTONE_CH];

    out->lm = lm;
    if (attempt_ab(ctx, p, Y, f, d, &out->evals)) {
        copy5(out->duty, d);
        out->mode = LEMTONE_EXACT;
        return;
    }

    // Desaturate along the OKLab line toward the target's own point on the white line.
    const float beta = beta_on(ctx->cw_ab, ctx->ww_ab, p);
    const float w[2] = {ctx->cw_ab[0] + beta * (ctx->ww_ab[0] - ctx->cw_ab[0]),
                        ctx->cw_ab[1] + beta * (ctx->ww_ab[1] - ctx->cw_ab[1])};
    if (attempt_ab(ctx, w, Y, f, d, &out->evals)) {
        copy5(out->duty, d);
        float lo = 0.0f, hi = 1.0f;  // saturation kept: lo feasible, hi not
        for (int s = 0; s < BISECT_STEPS; s++) {
            const float mid = 0.5f * (lo + hi);
            const float q[2] = {w[0] + mid * (p[0] - w[0]), w[1] + mid * (p[1] - w[1])};
            if (attempt_ab(ctx, q, Y, f, d, &out->evals)) {
                lo = mid;
                copy5(out->duty, d);
            } else {
                hi = mid;
            }
        }
        out->mode = LEMTONE_DESAT;
        out->desat = 1.0f - lo;
        return;
    }

    // Too bright for any colour: the brightest white-line point we can make.
    float lo = 0.0f, hi = 1.0f;
    for (int s = 0; s < BISECT_STEPS + 2; s++) {
        const float mid = 0.5f * (lo + hi);
        if (attempt_ab(ctx, w, mid * Y, f, d, &out->evals)) {
            lo = mid;
            copy5(out->duty, d);
        } else {
            hi = mid;
        }
    }
    out->mode = LEMTONE_CLAMP;
    out->desat = 1.0f;
    out->lm = lo * lm;
}

// ---- periods --------------------------------------------------------------------

void lemtone_periods(const lemtone_period_cfg_t *cfg, const float duty[LEMTONE_CH], float lm,
                     uint8_t out[LEMTONE_CH]) {
    uint8_t cap = cfg->n_max;
    for (int t = 0; t < cfg->n_tiers && t < LEMTONE_MAX_TIERS; t++) {
        if (lm > cfg->tier[t].lm_above) {
            cap = cfg->tier[t].n_cap;
            break;
        }
    }
    if (cap < cfg->n_min) cap = cfg->n_min;
    if (cap > cfg->n_max) cap = cfg->n_max;
    for (int i = 0; i < LEMTONE_CH; i++) {
        uint8_t n = cfg->n_min;
        if (duty[i] * (float)(1u << cfg->n_max) >= 0.5f) {  // else no whole tick even at n_max: off
            while (n < cap && duty[i] * (float)(1u << n) < (float)cfg->res_ticks) n++;
        }
        out[i] = n;
    }
}

// ---- quantize ---------------------------------------------------------------------

void lemtone_quantize(const lemtone_ctx_t *ctx, const float duty[LEMTONE_CH],
                      const uint8_t period_log2[LEMTONE_CH], lemtone_pwm_t *out) {
    for (int i = 0; i < LEMTONE_CH; i++) {
        const uint8_t n = period_log2[i];
        const float   L = (float)(1u << n);
        const float   pulse = ctx->m->pulse_us[i];
        float t = duty[i] > 0.0f ? duty[i] * L - pulse : 0.0f;
        const uint32_t tmax = (uint32_t)(ctx->m->duty_max * L);  // floor: never above the cap
        uint32_t ticks = t > 0.0f ? (uint32_t)(t + 0.5f) : 0u;
        if (ticks > tmax) ticks = tmax;
        out->period_log2[i] = n;
        out->ticks[i] = (uint16_t)ticks;
        out->cmd_duty[i] = (float)ticks / L;
        out->eff_duty[i] = ticks ? ((float)ticks + pulse) / L : 0.0f;
    }
}

void lemtone_compute(const lemtone_ctx_t *ctx, const lemtone_period_cfg_t *cfg,
                     float a, float b, float lm, float fidelity,
                     lemtone_solution_t *sol, lemtone_pwm_t *pwm) {
    lemtone_solve(ctx, a, b, lm, fidelity, sol);
    uint8_t periods[LEMTONE_CH];
    lemtone_periods(cfg, sol->duty, sol->lm, periods);
    lemtone_quantize(ctx, sol->duty, periods, pwm);
}
