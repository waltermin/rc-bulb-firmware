// variant_c3_rgb_cct.h -- ESP32-C3 board: RGB direct + tunable-white driver IC.
//
// Pin map read off the board's own PWM tuner firmware:
//     {"R": 1, "G": 5, "B": 4, "CCT": 0, "DIM": 6}  freq=5000
// All five are in the C3's safe GPIO set -- clear of the SPI flash (11-17), USB
// (18/19) and UART0 (20/21).
//
// R/G/B drive LED strings directly, exactly as on the ESP8266 bulb. CCT and DIM
// are INPUTS TO A DRIVER IC, not strings: CCT selects where between warm and cold
// the white pair sits, DIM sets that pair's brightness. pwm_mix.c turns the
// protocol's CW/WW into that pair.
//
// Two consequences the table encodes:
//   * rate_locked -- a driver IC reads duty as a level and expects a stable,
//     in-spec carrier. The 0x06 RawLightUpdate packet may retune the RGB channels
//     freely but must not retune these.
//   * phase 0 -- staggering phase reduces peak current across parallel LED
//     strings. It means nothing on a driver's control input.
//
// UNVERIFIED: the tuner ran these at 5000 Hz. The schedule model needs a
// power-of-two microsecond period, so 2^8 us = 3906 Hz is the closest available.
// Confirm that is inside the driver's accepted carrier range before shipping;
// if it is not, the driver inputs need a path outside the 2^n schedule.

#ifndef BULB_VARIANT_C3_RGB_CCT_H
#define BULB_VARIANT_C3_RGB_CCT_H

#include "config.h"

#define C3_GPIO_RED    1
#define C3_GPIO_GREEN  5
#define C3_GPIO_BLUE   4
#define C3_GPIO_CCT    0
#define C3_GPIO_DIM    6

#define C3_DRIVER_PERIOD_LOG2  8   // 2^8 us = 3906 Hz; see note above

// MEASURED on hardware: driving the CCT input at 0% duty lights the fixture
// COLD, so this driver's colour-temperature axis runs cold -> warm as duty
// rises. The opposite convention is at least as common, hence the flag rather
// than a hardcoded sign in the mixer.
#define VARIANT_CCT_ZERO_IS_COLD 1

static const pwm_channel_t s_channels[] = {
    { C3_GPIO_RED,   COLOR_R, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_RED,   PWM_PHASE_RED,   false },
    { C3_GPIO_GREEN, COLOR_G, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_GREEN, PWM_PHASE_GREEN, false },
    { C3_GPIO_BLUE,  COLOR_B, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_BLUE,  PWM_PHASE_BLUE,  false },
    { C3_GPIO_CCT,   0,       CH_ROLE_CCT,    C3_DRIVER_PERIOD_LOG2, 0.0f,            true  },
    { C3_GPIO_DIM,   0,       CH_ROLE_DIM,    C3_DRIVER_PERIOD_LOG2, 0.0f,            true  },
};

#define PWM_VARIANT_NAME "c3_rgb_cct"

#endif  // BULB_VARIANT_C3_RGB_CCT_H
