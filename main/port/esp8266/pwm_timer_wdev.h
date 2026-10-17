// pwm_timer_wdev.h -- ESP8266 backend: Wi-Fi WDEV/TSF0 timer at NMI level.
//
// Lifted verbatim from the original pwm_output.c. The register dance is the stock
// SDK PWM driver's, which is proven glitch-free alongside the Wi-Fi stack; do not
// "simplify" it. The TSF counter ticks at 1 us, which is also the schedule's unit,
// so PWM_TICKS_PER_US is 1 and no conversion happens on this target.

#ifndef BULB_PWM_TIMER_WDEV_H
#define BULB_PWM_TIMER_WDEV_H

#include <stdint.h>
#include "esp8266/eagle_soc.h"
#include "esp8266/gpio_struct.h"
#include "esp_attr.h"

#define PWM_TICKS_PER_US   1u
#define PWM_TMR_MIN_WAKE   2u   // SDK requires the relative compare be >= ~2 us

void pwm_tmr_init(void (*isr)(void));
void pwm_tmr_enable(void);

// TSF0's compare interrupt is cleared by the Wi-Fi layer before our callback
// runs, so there is nothing to acknowledge here.
static inline void IRAM_ATTR pwm_tmr_ack(void) { }

static inline uint32_t IRAM_ATTR pwm_tmr_now(void) {
    return REG_READ(WDEVTSF0_TIME_LO);
}

// Freeze TSF update -> disable compare -> zero counter -> set RELATIVE compare ->
// re-enable -> thaw. Zeroing is what makes the compare unconditionally future.
static inline void IRAM_ATTR pwm_tmr_arm_raw(uint32_t wake) {
    if (wake < PWM_TMR_MIN_WAKE) wake = PWM_TMR_MIN_WAKE;
    REG_WRITE(WDEVSLEEP0_CONF, REG_READ(WDEVSLEEP0_CONF) & ~WDEV_TSFUP0_ENA);
    REG_WRITE(WDEVTSF0TIMER_ENA, REG_READ(WDEVTSF0TIMER_ENA) & ~WDEV_TSF0TIMER_ENA);
    REG_WRITE(WDEVTSFSW0_LO, 0);
    REG_WRITE(WDEVTSF0_TIMER_LO, wake);
    REG_WRITE(WDEVTSF0TIMER_ENA, WDEV_TSF0TIMER_ENA);
    REG_WRITE(WDEVSLEEP0_CONF, REG_READ(WDEVSLEEP0_CONF) | WDEV_TSFUP0_ENA);
}

// The counter is about to be zeroed, so the returned target is simply the gap --
// that IS its value in the new epoch.
static inline uint32_t IRAM_ATTR pwm_tmr_arm_rebase(uint32_t target, uint32_t ahead) {
    int32_t wake = (int32_t)(target - REG_READ(WDEVTSF0_TIME_LO));
    if (wake < (int32_t)(PWM_TMR_MIN_WAKE + ahead)) {
        wake = (int32_t)(PWM_TMR_MIN_WAKE + ahead);
    }
    pwm_tmr_arm_raw((uint32_t)wake - ahead);
    return (uint32_t)wake;
}

// Bulk GPIO set/clear, one register write each -- the whole point of the
// state-byte lookup tables.
static inline void IRAM_ATTR pwm_gpio_set(uint32_t mask) { GPIO.out_w1ts = mask; }
static inline void IRAM_ATTR pwm_gpio_clr(uint32_t mask) { GPIO.out_w1tc = mask; }

#endif  // BULB_PWM_TIMER_WDEV_H
