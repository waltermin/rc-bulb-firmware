// pwm_mix.c -- logical colour -> physical channels.
//
// On a direct-drive board this is an index map and nothing more. On a board whose
// white pair hangs off a tunable-white driver IC, the protocol's independent
// (CW, WW) has to become the driver's (CCT, DIM), and that is a real projection
// with real losses. The maths:
//
// The driver does a LINEAR FLUX SPLIT at constant total drive. Measured with a
// spectrometer, holding DIM fixed and stepping CCT across its range:
//
//     cct   0.00   0.25   0.50   0.75   1.00      (1.00 = fully warm)
//     lum   44.5   40.7   36.3   31.7   27.0
//
// Luminance falls monotonically cold -> warm because the cold string is ~1.65x
// more luminous-efficient, not because total drive changes. The midpoint sits at
// the ENDPOINT AVERAGE (measured 1.016x, linear predicts 1.00, a constant-power
// driver would give 1.41), and the linear fit holds at every step to ~1.5%. The
// CCT input is therefore linear in the mix fraction -- no lookup table needed.
//
// So the inversion is a flux split, not a rotation:
//
//     dim = cw + ww                 total drive wanted
//     cct = ww / (cw + ww)          which fraction of it is warm
//
// An earlier version used cct = atan2(cw, ww), dim = hypot(cw, ww), assuming a
// constant-POWER driver. That was wrong and worth recording: since
// hypot(a,b) <= a+b, it UNDER-delivered, by 1/sqrt(2) (~29%) at 50/50, sagging
// to a minimum mid-range and correct only at the endpoints. The cheap test to
// repeat on any new fixture is the constant-DIM CCT sweep above: flat or
// endpoint-average means a flux split, a bulge in the middle means constant
// power and the norm changes back.
//
// Two properties survive the change, and they are why the mixer stays a leaf:
//
//  1. The duty curve and the max-power cap CANCEL out of cct. Both scale cw and
//     ww by the same factor, and cct depends only on their ratio. dim, being the
//     total, inherits the cap correctly. True only because the mixer runs AFTER
//     compute_duties(); mixing before the curve would need per-channel cap and
//     curve policy to stop the cap clipping the top of the CCT range.
//  2. Clipping is free. When the total exceeds 1 we clamp it, which leaves the
//     ratio -- and so the colour temperature -- untouched. Brightness is lost,
//     colour never is. The right trade for a lamp, and it needs no special case.
//
// What is unreachable: because total drive is capped at one string's worth,
// cw + ww <= 1. The reachable set is a TRIANGLE, not the unit square a
// five-string bulb gives. Pure cold and pure warm are exact; a request for both
// whites at full clamps to half of each. So what you lose is peak output in the
// MIDDLE of the CCT range -- a harder limit than the quarter disc the old model
// implied, and worth knowing when setting expectations for a scene.
//
// DIM is LINEAR IN FLUX, so feeding it linear light (post-curve) is correct and
// needs no inverse compensation. Measured at fixed cct, duty vs luminance
// normalised at 0.8:
//
//     duty   0.10  0.20  0.30  0.40  0.60  0.80
//     ratio  1.10  1.09  1.08  1.07  1.04  1.00     (1.00 == perfectly linear)
//
// A driver applying its own perceptual curve would read ~16x low at duty 0.10,
// not 10% high, so there is no double-correction to undo. The residual lift at
// low drive is ordinary LED efficacy droop -- they are slightly more efficient
// at lower current -- and at under 10% across an 8:1 range it is not worth
// compensating.

#include "pwm_mix.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void pwm_mix(color5_t d, const pwm_channel_t *ch, int n, float *out) {
    float cct = 0.0f, dim = 0.0f;

    bool need_white_pair = false;
    for (int i = 0; i < n; i++) {
        if (ch[i].role == CH_ROLE_CCT || ch[i].role == CH_ROLE_DIM) {
            need_white_pair = true;
            break;
        }
    }

    if (need_white_pair) {
        const float cw = d.ch[COLOR_CW];
        const float ww = d.ch[COLOR_WW];
        const float total = cw + ww;
        if (total <= 0.0f) {
            cct = 0.0f;   // white off; the ratio is undefined, so be explicit
            dim = 0.0f;
        } else {
            // Which end 0% duty means is the driver's convention, not ours.
#ifdef VARIANT_CCT_ZERO_IS_COLD
            cct = ww / total;
#else
            cct = cw / total;
#endif
            dim = total;
            if (dim > 1.0f) dim = 1.0f;   // clamp total, ratio survives
        }
    }

    for (int i = 0; i < n; i++) {
        switch (ch[i].role) {
        case CH_ROLE_CCT: out[i] = cct; break;
        case CH_ROLE_DIM: out[i] = dim; break;
        default:          out[i] = d.ch[ch[i].source]; break;
        }
    }
}
