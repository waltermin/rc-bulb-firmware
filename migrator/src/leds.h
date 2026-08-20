// Diagnostic LED channels for the migrator (Kauf RGBWW GPIO map).
// Colors: solid BLUE while migrating, solid GREEN on success, flashing RED on
// failure. NEVER drive any channel above LED_MAX (of 255) -- both analogWrite
// (cache-on) and startWaveform (cache-off IRAM) are clamped to it.
#pragma once

#define LED_R_PIN 4
#define LED_G_PIN 12
#define LED_B_PIN 14

#define LED_MAX 192            // hard cap on PWM level (of 255) -- do not exceed
#define LED_PERIOD_US 4000u    // 250 Hz, flicker-free

// High/low microseconds for a "solid" LED_MAX-duty waveform (startWaveform).
#define LED_ON_US  ((LED_MAX * LED_PERIOD_US) / 255u)   // ~3012 us
#define LED_OFF_US (LED_PERIOD_US - LED_ON_US)          // ~988 us
