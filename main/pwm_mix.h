// pwm_mix.h -- logical colour (R G B CW WW) -> the variant's physical channels.
#ifndef BULB_PWM_MIX_H
#define BULB_PWM_MIX_H

#include "variant.h"

// `duties` are already curve-mapped and max-power-capped. `out` receives one
// normalised duty per physical channel, in channel-table order.
void pwm_mix(color5_t duties, const pwm_channel_t *chans, int n, float *out);

#endif  // BULB_PWM_MIX_H
