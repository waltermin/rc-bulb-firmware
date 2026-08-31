// pwm_output.h — 5-channel LED PWM on a custom, self-contained software engine.
//
// Each channel has its own duty, phase, and period. The engine runs on the
// ESP8266 FRC1 hardware timer (radio-independent, 200 ns ticks) and compiles the
// channels into one repeatable edge table walked by an IRAM-resident ISR, so the
// LEDs light immediately at boot and keep running through OTA flash writes.
//
// The drive path is: pwm_output_set() runs the duty-response curve + max-power cap
// (levels_to_duties()), then pwm_compile() (in pwm_schedule.c) turns per-channel
// {duty, phase, period} into the edge table. To experiment with duty curves,
// change levels_to_duties(); to experiment with the schedule/edge model, change
// pwm_schedule.c — nothing else needs to move.

#ifndef BULB_PWM_OUTPUT_H
#define BULB_PWM_OUTPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the PWM engine: configure the channel GPIOs as outputs, build the
// state->GPIO lookup tables, start the FRC1 timer, and apply the default color.
// Unlike the old SDK-PWM path this needs no radio, so the LEDs light here — call
// it early in app_main.
void pwm_output_init(void);

// Set all five channels from normalized levels in [0.0, 1.0] (clamped to range).
// The gamma/perceptual curve, max-power cap, per-channel phase and period are all
// applied internally; the new frame is published to the ISR glitch-free.
void pwm_output_set(float r, float g, float b, float ww, float cw);

// Apply the compile-time / NVS DEFAULT_* color. Idempotent.
void pwm_output_set_default(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_PWM_OUTPUT_H
