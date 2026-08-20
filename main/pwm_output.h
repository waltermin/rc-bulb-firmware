// pwm_output.h — 5-channel LED PWM, matching stock bulb behavior
// (250 Hz base, 80% max power, gamma 2.8, phase-aligned channels).
//
// The drive algorithm (how a color becomes concrete PWM period/duty/phase) is
// isolated in compute_frame() inside pwm_output.c — that is the single place to
// change to experiment with different duty curves, phase layouts, or a dynamic
// period.

#ifndef BULB_PWM_OUTPUT_H
#define BULB_PWM_OUTPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the PWM peripheral (GPIO config, phases, default color) and issue
// a first start. Call this early in app_main. NOTE: the SDK PWM driver clocks
// its ISR off the Wi-Fi (WDEV/TSF0) hardware timer, which does not tick until
// the radio is started, so this early start does not yet produce light on its
// own — pwm_output_start_after_radio() below must be called once the radio is
// up to actually begin output.
void pwm_output_init(void);

// Re-arm the PWM driver once the radio (esp_wifi_start) is running, so its
// WDEV-timer ISR actually fires and the LEDs light at the default color. Call
// exactly once, right after Wi-Fi init. Without this the LEDs stay dark until
// some later event happens to kick the driver.
void pwm_output_start_after_radio(void);

// Set all five channels from normalized duty cycles in [0.0, 1.0] (values are
// clamped to that range). The gamma curve, max-power cap, phase offsets, and
// PWM period are all applied internally by the drive algorithm.
void pwm_output_set(float r, float g, float b, float ww, float cw);

// Apply the compile-time DEFAULT_* color. Idempotent.
void pwm_output_set_default(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_PWM_OUTPUT_H
