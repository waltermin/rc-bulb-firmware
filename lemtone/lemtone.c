// lemtone.c — Lemtone target -> per-channel duty + PWM period (see lemtone.h).
//
// The solve:
//   1. Closed form on a linear model XYZ = sum_i d_i P_i, as lemtone.html does: the
//      white level `we` is split CW/WW so the white mix lands where the target projects
//      onto the CW-WW xy line (a duty share, weighted by each white's X+Y+Z per unit duty),
//      r,g,b come from the 3x3 primary inverse, and the range of `we` that keeps every
//      duty in [0, duty_max] is found directly; fidelity picks a point in that range.
//   2. Frozen coefficients: P_i starts as each channel's output per unit duty at
//      REF_DUTY, then is re-evaluated at the duties just found (interaction and the
//      duty-dependent colour included) and step 1 runs again, until the full model
//      lands on the target. Because step 1 is redone with the true local P_i, the
//      fidelity choice holds for the real model too (WW alone stays WW alone).
//   3. Infeasible target: bisect the saturation along the OKLab line toward the white the
//      whites alone make at the target's place on their line and at its lumens (always
//      reachable, unlike the straight OKLab line between CW and WW, which runs ~0.01 off
//      their mix midway). So the colour desaturates only as far as R, G, B run out of
//      headroom, more as the lumens grow. If even that point cannot reach the lumens,
//      bisect the lumens there.
//
// Targets arrive as OKLab chromaticity + lumens and are turned into XYZ once; the solve
// itself runs in XYZ, where the model is (nearly) linear.

#include "lemtone.h"

#define REF_DUTY     0.4f   // where the closed form's primaries are taken
#define PHASE1_ROUNDS 6     // rounds without the interaction term
#define FIX_ROUNDS   12     // rounds with it
#define FIX_TOL      2e-3f  // converged when every XYZ error < FIX_TOL * Y
#define BISECT_STEPS 10
#define WHITE_ROUNDS 8      // white_only: luminance ratio iterations
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

// Duty at which channel i's knots are read: its own, or the pair's DIM (white_by_dim).
static float knot_duty(const lemtone_model_t *m, int i, const float d[LEMTONE_CH]) {
    if (m->white_by_dim && (i == LEMTONE_WW || i == LEMTONE_CW)) return d[LEMTONE_WW] + d[LEMTONE_CW];
    return d[i];
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
        unit_xyz(ctx->m, i, knot_duty(ctx->m, i, d), u);
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


// Position t along the xy line CW -> WW -> WW share of the white duty that lands there (the
// mix is weighted by each white's X+Y+Z per unit duty).
static float duty_split(const float P[LEMTONE_CH][3], float t) {
    const float sc = P[LEMTONE_CW][0] + P[LEMTONE_CW][1] + P[LEMTONE_CW][2];
    const float sw = P[LEMTONE_WW][0] + P[LEMTONE_WW][1] + P[LEMTONE_WW][2];
    const float den = sw * (1.0f - t) + t * sc;
    return den > 0.0f ? t * sc / den : t;
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
    // the duty share that puts the white mix on p's place along the CW-WW xy line
    const float beta = duty_split(P, beta_on(cw, ww, p));
    for (int c = 0; c < 3; c++) {
        wmix[c] = (1.0f - beta) * P[LEMTONE_CW][c] + beta * P[LEMTONE_WW][c];
    }
    mul3(ainv, tgt, a);   // rgb(we) = a - we * b
    mul3(ainv, wmix, b);

    float lo = 0.0f, hi = dmax / maxf(maxf(beta, 1.0f - beta), EPS);
    if (ctx->m->white_sum_max > 0.0f) hi = minf(hi, ctx->m->white_sum_max);  // we = ww + cw
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
        unit_xyz(ctx->m, i, knot_duty(ctx->m, i, d), P[i]);
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
static void start_primaries(const lemtone_ctx_t *ctx, float P[LEMTONE_CH][3]) {
    for (int i = 0; i < LEMTONE_CH; i++) {
        for (int c = 0; c < 3; c++) P[i][c] = ctx->prim[i][c];
    }
}

static bool attempt(const lemtone_ctx_t *ctx, const float tgt[3], float f, float d[LEMTONE_CH],
                    uint16_t *evals) {
    const float Y = tgt[1];
    if (!(Y > 0.0f) || tgt[0] < 0.0f || tgt[2] < 0.0f) return false;  // not a real colour
    float p[2];
    xy_of(tgt, p);
    float P[LEMTONE_CH][3];
    start_primaries(ctx, P);
    bool feasible;
    if (!rounds(ctx, tgt, p, Y, f, false, PHASE1_ROUNDS, P, d, &feasible, evals) && !feasible) {
        // infeasible without sag: start phase 2 from scratch rather than give up
        start_primaries(ctx, P);
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

// Whites alone with WW duty share beta, at luminance Y or as close as the cap allows. Returns
// the luminance delivered. Ratio iteration: Y is nearly linear in the white level.
static float white_only(const lemtone_ctx_t *ctx, float beta, float Y, float d[LEMTONE_CH],
                        uint16_t *evals) {
    float cap = ctx->m->duty_max / maxf(maxf(beta, 1.0f - beta), EPS);
    if (ctx->m->white_sum_max > 0.0f) cap = minf(cap, ctx->m->white_sum_max);
    float xyz[3];
    for (int i = 0; i < LEMTONE_CH; i++) d[i] = 0.0f;
    d[LEMTONE_WW] = cap * beta;
    d[LEMTONE_CW] = cap * (1.0f - beta);
    lemtone_forward(ctx, d, xyz);
    (*evals)++;
    if (!(xyz[1] > Y)) return xyz[1];
    float we = cap * Y / xyz[1];
    for (int k = 0; k < WHITE_ROUNDS; k++) {
        d[LEMTONE_WW] = we * beta;
        d[LEMTONE_CW] = we * (1.0f - beta);
        lemtone_forward(ctx, d, xyz);
        (*evals)++;
        if (absf(xyz[1] - Y) < FIX_TOL * Y || !(xyz[1] > 0.0f)) break;
        we = minf(cap, we * Y / xyz[1]);
    }
    d[LEMTONE_WW] = we * beta;
    d[LEMTONE_CW] = we * (1.0f - beta);
    return Y;
}

// OKLab chromaticity of the whites alone at p's place on the CW-WW xy line, at luminance Y
// (or the brightest they make there): where an unreachable colour desaturates to.
static void white_anchor(const lemtone_ctx_t *ctx, const float p[2], float Y, float w[2],
                         uint16_t *evals) {
    float X[3], xy[2], cw[2], ww[2], d[LEMTONE_CH];
    xy_of(ctx->prim[LEMTONE_CW], cw);
    xy_of(ctx->prim[LEMTONE_WW], ww);
    float t;
    if (lemtone_ab_to_xyz(p[0], p[1], X)) {
        xy_of(X, xy);
        t = beta_on(cw, ww, xy);
    } else {
        t = beta_on(ctx->cw_ab, ctx->ww_ab, p);  // imaginary: project in OKLab instead
    }
    white_only(ctx, duty_split(ctx->prim, t), Y, d, evals);
    lemtone_forward(ctx, d, X);
    (*evals)++;
    lemtone_xyz_to_ab(X, w);
}

static void solve_free(const lemtone_ctx_t *ctx, float a, float b, float lm, float fidelity,
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

    // Desaturate toward the white the whites themselves make at the target's place on the
    // CW-WW line and at its lumens: R, G, B pull the colour off that point as far as they can,
    // which shrinks as the lumens grow.
    float w[2];
    white_anchor(ctx, p, Y, w, &out->evals);
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

// ---- driver whites on their tick grid ------------------------------------------------

// R, G, B for XYZ tgt with the whites held at d[WW], d[CW] (frozen-coefficient rounds, the
// interaction included). d[R..B] is the starting guess and receives the result, clamped to
// [0, duty_max]. Returns how far the result misses tgt (largest XYZ error).
static float rgb_for_whites(const lemtone_ctx_t *ctx, const float tgt[3], float d[LEMTONE_CH],
                            uint16_t *evals) {
    const float dmax = ctx->m->duty_max;
    float P[LEMTONE_CH][3], ainv[3][3], rest[3], rgb[3];
    for (int it = 0; it < FIX_ROUNDS; it++) {
        local_primaries(ctx, d, true, P);
        (*evals)++;
        if (!inv_rgb(P, ainv)) break;
        for (int c = 0; c < 3; c++) {
            rest[c] = tgt[c] - d[LEMTONE_WW] * P[LEMTONE_WW][c] - d[LEMTONE_CW] * P[LEMTONE_CW][c];
        }
        mul3(ainv, rest, rgb);
        float step = 0.0f;
        for (int k = 0; k < 3; k++) {
            const float v = clampf(rgb[k], 0.0f, dmax);
            step = maxf(step, absf(v - d[k]));
            d[k] = v;
        }
        if (step < 1e-5f) break;
    }
    float X[3], err = 0.0f;
    lemtone_forward(ctx, d, X);
    (*evals)++;
    for (int c = 0; c < 3; c++) err = maxf(err, absf(X[c] - tgt[c]));
    return err;
}

// Driver whites (white_grid_log2): put the white pair on its DIM / CCT grid and redo R, G, B
// for the light the unsnapped solution makes, so the dim end of the white range does not land
// in steps (at 8 bits one DIM step is 1-17% of the whites' light below ~100 lm).
// Candidates in order: the nearest DIM step, the other neighbour, whites off. The first that
// R, G, B complete to the solver's tolerance wins, else the one that misses least.
static void snap_whites(const lemtone_ctx_t *ctx, lemtone_solution_t *out) {
    const lemtone_model_t *m = ctx->m;
    const float L = (float)(1u << m->white_grid_log2);
    const float total = out->duty[LEMTONE_WW] + out->duty[LEMTONE_CW];
    if (!(total > 0.0f)) return;
    float tgt[3];
    lemtone_forward(ctx, out->duty, tgt);
    out->evals++;
    const float cap = m->white_sum_max > 0.0f ? minf(m->white_sum_max, 1.0f) : 1.0f;
    const float c = (float)(int)(out->duty[LEMTONE_WW] / total * L + 0.5f) / L;
    const float x = total * L;
    int nearest = (int)(x + 0.5f), other = (float)nearest > x ? nearest - 1 : nearest + 1;
    const int dmin = m->white_dim_min;
    if (nearest > 0 && nearest < dmin) {   // the driver's unusable bottom: dmin or off
        nearest = (float)dmin - x < x ? dmin : 0;
        other = nearest ? 0 : dmin;
    } else if (other > 0 && other < dmin) {
        other = nearest == dmin ? 0 : dmin;
    }
    const int cand[3] = {nearest, other, 0};
    float best[LEMTONE_CH], best_err = -1.0f;
    for (int k = 0; k < 3; k++) {
        const int t = cand[k];
        if (t < 0 || (float)t > cap * L || (k == 2 && (nearest == 0 || other == 0))) continue;
        float d[LEMTONE_CH];
        copy5(d, out->duty);
        const float D = (float)t / L;
        d[LEMTONE_WW] = D * c;
        d[LEMTONE_CW] = D * (1.0f - c);
        const float err = rgb_for_whites(ctx, tgt, d, &out->evals);
        if (best_err < 0.0f || err < best_err) {
            copy5(best, d);
            best_err = err;
        }
        if (err < FIX_TOL * tgt[1]) break;
    }
    if (best_err >= 0.0f) copy5(out->duty, best);
}

void lemtone_solve(const lemtone_ctx_t *ctx, float a, float b, float lm, float fidelity,
                   lemtone_solution_t *out) {
    solve_free(ctx, a, b, lm, fidelity, out);
    if (ctx->m->white_grid_log2 && (out->mode == LEMTONE_EXACT || out->mode == LEMTONE_DESAT)) {
        snap_whites(ctx, out);
    }
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

float lemtone_pulse_us(const lemtone_model_t *m, int i, uint8_t n) {
    if (!m->pulse_by_period) return m->pulse_us[i];
    int k = (int)n - LEMTONE_PULSE_N0;
    if (k < 0) k = 0;
    if (k > LEMTONE_PULSE_NN - 1) k = LEMTONE_PULSE_NN - 1;
    return m->pulse_n[i][k];
}

// Driver whites: DIM = ww + cw and CCT = ww / (ww + cw), each a whole number of 2^-n steps,
// then back to the logical pair the mixer expects (it recomputes the same two ratios, and the
// PWM rounds them back to these steps).
static void quantize_white_pair(const lemtone_model_t *m, const float duty[LEMTONE_CH],
                                lemtone_pwm_t *out) {
    const uint8_t  n = m->white_grid_log2;
    const float    L = (float)(1u << n);
    const float    total = duty[LEMTONE_WW] + duty[LEMTONE_CW];
    const float    cap = m->white_sum_max > 0.0f ? minf(m->white_sum_max, 1.0f) : 1.0f;
    const uint32_t dmax = (uint32_t)(cap * L);
    uint32_t dim = total > 0.0f ? (uint32_t)(total * L + 0.5f) : 0u;
    if (dim > dmax) dim = dmax;
    uint32_t cct = dim ? (uint32_t)(duty[LEMTONE_WW] / total * L + 0.5f) : 0u;
    if (cct > (uint32_t)L) cct = (uint32_t)L;
    const float D = (float)dim / L, c = (float)cct / L;
    out->period_log2[LEMTONE_WW] = out->period_log2[LEMTONE_CW] = n;
    out->ticks[LEMTONE_CW] = (uint16_t)dim;
    out->ticks[LEMTONE_WW] = (uint16_t)cct;
    out->cmd_duty[LEMTONE_WW] = out->eff_duty[LEMTONE_WW] = D * c;
    out->cmd_duty[LEMTONE_CW] = out->eff_duty[LEMTONE_CW] = D * (1.0f - c);
}

void lemtone_quantize(const lemtone_ctx_t *ctx, const float duty[LEMTONE_CH],
                      const uint8_t period_log2[LEMTONE_CH], lemtone_pwm_t *out) {
    for (int i = 0; i < LEMTONE_CH; i++) {
        const uint8_t n = period_log2[i];
        const float   L = (float)(1u << n);
        const float   pulse = lemtone_pulse_us(ctx->m, i, n);
        // under half a tick of light is off; with a negative pulse (light lost per pulse) a
        // near-zero duty would otherwise still get the ~|pulse| ticks the pulse eats
        float t = duty[i] * L >= 0.5f ? duty[i] * L - pulse : 0.0f;
        const uint32_t tmax = (uint32_t)(ctx->m->duty_max * L);  // floor: never above the cap
        uint32_t ticks = t > 0.0f ? (uint32_t)(t + 0.5f) : 0u;
        if (ticks > tmax) ticks = tmax;
        out->period_log2[i] = n;
        out->ticks[i] = (uint16_t)ticks;
        out->cmd_duty[i] = (float)ticks / L;
        out->eff_duty[i] = ticks ? maxf(0.0f, (float)ticks + pulse) / L : 0.0f;
    }
    if (ctx->m->white_grid_log2) quantize_white_pair(ctx->m, duty, out);
}

void lemtone_compute(const lemtone_ctx_t *ctx, const lemtone_period_cfg_t *cfg,
                     float a, float b, float lm, float fidelity,
                     lemtone_solution_t *sol, lemtone_pwm_t *pwm) {
    lemtone_solve(ctx, a, b, lm, fidelity, sol);
    uint8_t periods[LEMTONE_CH];
    lemtone_periods(cfg, sol->duty, sol->lm, periods);
    lemtone_quantize(ctx, sol->duty, periods, pwm);
}
