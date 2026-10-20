// pwm_hw.h -- seam for channels a PWM peripheral drives instead of the software engine.
//
// A variant marks such channels hw_pwm (variants/variant.h) and defines VARIANT_HW_PWM;
// the chip's port supplies the implementation (port/esp32c3/pwm_hw_ledc.c). Only
// pwm_output.c calls these, from task context.
//
// The software engine exists because a 1 us timer is the only fine clock some chips
// offer; a channel that needs a fixed carrier with more duty resolution than 1 us ticks
// give (the panel's driver-IC inputs) is better off on hardware PWM, which also needs
// no edge table and no ISR.

#ifndef BULB_PWM_HW_H
#define BULB_PWM_HW_H

#include "variant.h"

// Configure every hw_pwm channel of the table, all at duty 0.
void pwm_hw_init(const pwm_channel_t *chans, int n);

// Channel i of the table (an hw_pwm one; others are ignored) to `duty` in [0, 1],
// rounded to the peripheral's resolution as lroundf(duty * 2^bits). Takes effect at the
// next carrier period. Repeats of the current value cost nothing.
void pwm_hw_set(int i, float duty);

#endif  // BULB_PWM_HW_H
