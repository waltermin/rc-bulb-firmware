// variant.h -- what the light hardware actually looks like.
//
// A variant describes the physical channel set: which GPIO each channel drives,
// what that channel MEANS, its PWM rate and phase. The engine and the schedule
// compiler are variant-agnostic; only this table and the mixer know the topology.
//
// Two shipped today:
//   kauf_rgbww   ESP8266 bulb -- five direct LED strings (R G B CW WW)
//   c3_rgb_cct   ESP32-C3 board -- R G B direct, plus a tunable-white driver IC
//                fed by a colour-temperature input and a brightness input
//
// The control protocol never changes: the wire, the controller and the duty curve
// all speak R/G/B/CW/WW on every variant. Translating that to whatever the board
// actually has happens once, at the output edge, in pwm_mix.c.

#ifndef BULB_VARIANT_H
#define BULB_VARIANT_H

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"

// Logical colour components -- the five arguments of pwm_output_set(), fixed
// order. These are what the protocol and the duty curve work in, on every board.
enum {
    COLOR_R = 0,
    COLOR_G,
    COLOR_B,
    COLOR_WW,
    COLOR_CW,
    COLOR_COUNT,
};

// A 5-tuple of normalised floats indexed by COLOR_*, used for both input levels
// and computed duties as they flow through the duty math.
typedef struct {
    float ch[COLOR_COUNT];
} color5_t;

// What a physical channel is wired to.
typedef enum {
    CH_ROLE_DIRECT = 0,  // an LED string: duty IS brightness
    CH_ROLE_CCT,         // driver-IC colour-temperature input: duty is a POSITION
    CH_ROLE_DIM,         // driver-IC brightness input for the white pair
} pwm_ch_role_t;

typedef struct {
    uint32_t gpio;
    uint8_t  source;       // COLOR_* feeding this channel; DIRECT roles only
    uint8_t  role;         // pwm_ch_role_t
    uint8_t  period_log2;  // n; PWM period = 2^n us
    float    base_phase;   // degrees, (-180, 180]
    bool     rate_locked;  // ignore per-channel rate changes from 0x06 packets
} pwm_channel_t;

// Variant selection defaults from the target but can be forced with -D.
#if !defined(LIGHT_VARIANT_KAUF_RGBWW) && !defined(LIGHT_VARIANT_C3_RGB_CCT)
#  if defined(CONFIG_IDF_TARGET_ESP8266)
#    define LIGHT_VARIANT_KAUF_RGBWW 1
#  elif defined(CONFIG_IDF_TARGET_ESP32C3)
#    define LIGHT_VARIANT_C3_RGB_CCT 1
#  else
#    error "no light variant for this target"
#  endif
#endif

#if defined(LIGHT_VARIANT_KAUF_RGBWW)
#  include "variant_kauf_rgbww.h"
#elif defined(LIGHT_VARIANT_C3_RGB_CCT)
#  include "variant_c3_rgb_cct.h"
#endif

#endif  // BULB_VARIANT_H
