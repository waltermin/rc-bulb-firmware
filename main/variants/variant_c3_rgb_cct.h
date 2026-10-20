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
// Three consequences the table encodes:
//   * rate_locked -- a driver IC reads duty as a level and expects a stable,
//     in-spec carrier. The 0x06 RawLightUpdate packet may retune the RGB channels
//     freely but must not retune these.
//   * phase 0 -- staggering phase reduces peak current across parallel LED
//     strings. It means nothing on a driver's control input.
//   * hw_pwm -- CCT and DIM run on the C3's LEDC peripheral, not the 1 us software
//     engine: a fixed carrier at VARIANT_HW_PWM_BITS of duty resolution (12 bits: 4096
//     steps, against the engine's 256 at this carrier).
//
// Carrier: 3906.25 Hz, exactly what the panel was calibrated at (the first port drove
// these from the software engine at 2^8 us). The driver turns DIM into a steady LED
// current (calibration: mid-CCT brightens with DIM the way a split current does), so the
// carrier frequency is part of the calibration; do not change it without re-measuring
// the whites. The board's own tuner firmware ran 5000 Hz.
//
// UNVERIFIED: whether the driver resolves the finer steps. If it decodes only 8 bits
// internally, fades would still step every 16 counts; then set the model's grid back to
// 8 (calibration/export_panel_model.py WHITE_GRID_LOG2) and leave the hardware as is.

#ifndef BULB_VARIANT_C3_RGB_CCT_H
#define BULB_VARIANT_C3_RGB_CCT_H

#include "config.h"

#define C3_GPIO_RED    1
#define C3_GPIO_GREEN  5
#define C3_GPIO_BLUE   4
#define C3_GPIO_CCT    0
#define C3_GPIO_DIM    6

// The driver inputs on LEDC (port/esp32c3/pwm_hw_ledc.c). 80 MHz APB / 5 / 4096 =
// 3906.25 Hz exactly: the driver asks freq_hz 3906 and the divider comes out at exactly 5.
#define VARIANT_HW_PWM        1
#define VARIANT_HW_PWM_BITS        12     // = lemtone white_grid_log2 of the panel model
#define VARIANT_HW_PWM_FREQ_HZ     3906
#define C3_DRIVER_PERIOD_LOG2 8      // 2^8 us: the same carrier, for code that reads a period

// MEASURED on hardware: driving the CCT input at 0% duty lights the fixture
// COLD, so this driver's colour-temperature axis runs cold -> warm as duty
// rises. The opposite convention is at least as common, hence the flag rather
// than a hardcoded sign in the mixer.
#define VARIANT_CCT_ZERO_IS_COLD 1

static const pwm_channel_t s_channels[] = {
    { C3_GPIO_RED,   COLOR_R, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_RED,   PWM_PHASE_RED,   false, false },
    { C3_GPIO_GREEN, COLOR_G, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_GREEN, PWM_PHASE_GREEN, false, false },
    { C3_GPIO_BLUE,  COLOR_B, CH_ROLE_DIRECT, PWM_PERIOD_LOG2_BLUE,  PWM_PHASE_BLUE,  false, false },
    { C3_GPIO_CCT,   0,       CH_ROLE_CCT,    C3_DRIVER_PERIOD_LOG2, 0.0f,            true,  true  },
    { C3_GPIO_DIM,   0,       CH_ROLE_DIM,    C3_DRIVER_PERIOD_LOG2, 0.0f,            true,  true  },
};

#define PWM_VARIANT_NAME "c3_rgb_cct"

#endif  // BULB_VARIANT_C3_RGB_CCT_H
