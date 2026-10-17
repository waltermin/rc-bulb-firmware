// pwm_timer.h -- the one hardware seam in the PWM engine.
//
// The engine (pwm_output.c) walks a compiled edge table and places each edge at
// an exact tick. Everything about HOW time is read and how the next interrupt is
// armed lives behind this interface, so the same ISR body serves both targets.
//
// Two backends, with genuinely different timebase models:
//
//   ESP8266 -- Wi-Fi WDEV/TSF0 at NMI level, 1 us ticks. Arming follows the stock
//     SDK PWM's exact sequence (freeze TSF update -> disable compare -> ZERO the
//     counter -> set a RELATIVE compare -> re-enable -> thaw). The zeroing is not
//     incidental: it is what guarantees the compare always lands in the future,
//     and it is the sequence proven to coexist with the Wi-Fi stack. Every arm
//     therefore starts a NEW epoch in which now() restarts from 0.
//
//   ESP32-C3 -- systimer UNIT0 (free-running, 16 MHz) with alarm TARGET1, on a
//     CPU interrupt at priority 4. UNIT0 is shared with esp_timer and the
//     FreeRTOS tick, so it must never be zeroed or reset; the backend sets an
//     ABSOLUTE compare instead and the epoch never changes.
//
// Reconciling those is why arming is exposed as ONE primitive, pwm_tmr_arm_rebase(),
// rather than a separate "read now" + "set compare". The caller hands it the
// target it wants in the CURRENT epoch and gets back that same target expressed
// in whatever epoch exists after arming -- which the ESP8266 backend may have
// reset to zero and the C3 backend leaves alone. The ISR just does:
//
//     s_target = pwm_tmr_arm_rebase(s_target, PWM_AHEAD);
//
// and stays correct on both.
//
// Contract each backend must provide:
//   PWM_TICKS_PER_US    timer ticks per microsecond (schedules are in us)
//   PWM_TMR_MIN_WAKE    shortest relative compare the hardware accepts, in ticks
//   pwm_tmr_init(isr)   prepare hardware + bind the ISR; do NOT start counting
//   pwm_tmr_enable()    enable the interrupt (called once the first edge is armed)
//   pwm_tmr_ack()       acknowledge/clear the pending alarm; first thing in the ISR
//   pwm_tmr_now()       current time, low 32 bits, in ticks
//   pwm_tmr_arm_rebase(target, ahead)  see above
//
// now()/ack()/arm_rebase() are on the edge hot path and must be inlined and IRAM
// safe, so they are static inline in the backend header rather than real calls.

#ifndef BULB_PWM_TIMER_H
#define BULB_PWM_TIMER_H

#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#if defined(CONFIG_IDF_TARGET_ESP8266)
#  include "pwm_timer_wdev.h"
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#  include "pwm_timer_systimer.h"
#else
#  error "no PWM timer backend for this target"
#endif

// Critical sections differ in shape: ESP8266's take no argument, the C3's need a
// spinlock. NOTE for the C3: the PWM ISR runs at priority 4, ABOVE the threshold
// portENTER_CRITICAL raises -- so this does NOT exclude our own ISR there. It is
// not supposed to. The schedule handoff is lock-free by construction (see
// publish_reqs), and this only serialises task-level producers.
#if defined(CONFIG_IDF_TARGET_ESP8266)
#  define PWM_ENTER_CRITICAL()  portENTER_CRITICAL()
#  define PWM_EXIT_CRITICAL()   portEXIT_CRITICAL()
#else
extern portMUX_TYPE g_pwm_mux;
#  define PWM_ENTER_CRITICAL()  portENTER_CRITICAL(&g_pwm_mux)
#  define PWM_EXIT_CRITICAL()   portEXIT_CRITICAL(&g_pwm_mux)
#endif

// Does this backend's timer only run once the radio is up? True on the ESP8266,
// whose engine rides the Wi-Fi TSF counter; false on the C3, where systimer ticks
// from boot and real PWM can start before Wi-Fi is touched.
#if defined(CONFIG_IDF_TARGET_ESP8266)
#  define PWM_TIMER_NEEDS_RADIO 1
#else
#  define PWM_TIMER_NEEDS_RADIO 0
#endif

// Schedule ticks are microseconds (the wire format's period_log2 is in us);
// convert to the backend's native tick rate.
#define PWM_US_TO_TICKS(us) ((uint32_t)(us) * PWM_TICKS_PER_US)

#endif  // BULB_PWM_TIMER_H
