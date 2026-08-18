// pwm_output.h — 5-channel LED PWM, matching stock bulb behavior
// (250 Hz, 80% max power, gamma 2.8, phase-aligned channels).

#ifndef BULB_PWM_OUTPUT_H
#define BULB_PWM_OUTPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the PWM peripheral and start output. Call this as the very first
// thing in app_main so the LEDs light up before any radio work.
void pwm_output_init(void);

// Set all five channels from u8 values (gamma + max_power applied internally).
void pwm_output_set(uint8_t r, uint8_t g, uint8_t b, uint8_t ww, uint8_t cw);

// Apply the compile-time DEFAULT_* color. Idempotent.
void pwm_output_set_default(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_PWM_OUTPUT_H
