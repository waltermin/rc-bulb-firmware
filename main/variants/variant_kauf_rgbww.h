// variant_kauf_rgbww.h -- the original KAUF RGBWW bulb (ESP8266).
//
// Five independent LED strings, GPIOs and phase offsets from the stock
// kauf-bulb.yaml. The phase offsets stagger current draw across the strings;
// keeping them matches stock behaviour and lowers peak PSU ripple.
//
// OMIT_I2C_PINS: on the bench devboard GPIO12/14 are borrowed for I2C, so green
// and blue are dropped and their pins left untouched (high-Z after reset).

#ifndef BULB_VARIANT_KAUF_RGBWW_H
#define BULB_VARIANT_KAUF_RGBWW_H

#include "config.h"

static const pwm_channel_t s_channels[] = {
#if OMIT_I2C_PINS
    { PWM_GPIO_CW,    COLOR_CW, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_CW,    PWM_PHASE_CW,    false },
    { PWM_GPIO_RED,   COLOR_R,  CH_ROLE_DIRECT, PWM_PERIOD_LOG2_RED,   PWM_PHASE_RED,   false },
    { PWM_GPIO_WW,    COLOR_WW, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_WW,    PWM_PHASE_WW,    false },
#else
    { PWM_GPIO_CW,    COLOR_CW, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_CW,    PWM_PHASE_CW,    false },
    { PWM_GPIO_RED,   COLOR_R,  CH_ROLE_DIRECT, PWM_PERIOD_LOG2_RED,   PWM_PHASE_RED,   false },
    { PWM_GPIO_GREEN, COLOR_G,  CH_ROLE_DIRECT, PWM_PERIOD_LOG2_GREEN, PWM_PHASE_GREEN, false },
    { PWM_GPIO_BLUE,  COLOR_B,  CH_ROLE_DIRECT, PWM_PERIOD_LOG2_BLUE,  PWM_PHASE_BLUE,  false },
    { PWM_GPIO_WW,    COLOR_WW, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_WW,    PWM_PHASE_WW,    false },
#endif
};

#define PWM_VARIANT_NAME "kauf_rgbww"

#endif  // BULB_VARIANT_KAUF_RGBWW_H
