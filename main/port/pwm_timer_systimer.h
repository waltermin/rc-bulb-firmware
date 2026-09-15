// pwm_timer_systimer.h -- ESP32-C3 backend: systimer UNIT0 + alarm TARGET1,
// on a CPU interrupt reprogrammed to priority 4.
//
// UNIT0 is the chip's shared free-running 16 MHz counter -- esp_timer and the
// FreeRTOS tick both ride it -- so we only ever READ it and never reset it.
// Edges are placed with ABSOLUTE compare values; there is no epoch reset here,
// unlike the ESP8266 backend.
//
// Alarm allocation on this chip (confirmed by esp_intr_dump on hardware):
//   TARGET0 -> esp_timer      TARGET2 -> FreeRTOS tick      TARGET1 -> free

#ifndef BULB_PWM_TIMER_SYSTIMER_H
#define BULB_PWM_TIMER_SYSTIMER_H

#include <stdint.h>
#include "esp_attr.h"
#include "hal/systimer_ll.h"
#include "soc/systimer_struct.h"
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "soc/soc.h"

#define PWM_TICKS_PER_US   16u   // systimer counts at 16 MHz -> 62.5 ns/tick
#define PWM_TMR_MIN_WAKE   16u   // 1 us floor; the alarm self-compensates if missed

#define PWM_ST_UNIT        0
#define PWM_ST_ALARM       1

// THE number that makes this port work. IDF's RISC-V critical sections raise the
// interrupt threshold to RVHAL_EXCM_LEVEL-1 == 3, so priority 4 is the lowest
// that still fires inside portENTER_CRITICAL. Do not lower it: at 3 the ISR is
// masked and edges land up to a full critical section late (measured ~49 us).
#define PWM_INTR_PRIORITY  4

void pwm_tmr_init(void (*isr)(void));
void pwm_tmr_enable(void);

// Deliberately empty. The alarm latch is cleared inside pwm_tmr_arm_rebase(),
// AFTER the next target is programmed -- see the note there. Clearing it here,
// at ISR entry, is what caused a priority-4 interrupt storm.
static inline void IRAM_ATTR pwm_tmr_ack(void) { }

// Snapshot latches both halves together, so lo/hi read order does not matter.
static inline uint64_t IRAM_ATTR pwm_tmr_now64(void) {
    systimer_ll_counter_snapshot(&SYSTIMER, PWM_ST_UNIT);
    while (!systimer_ll_is_counter_value_valid(&SYSTIMER, PWM_ST_UNIT)) { }
    uint32_t lo = systimer_ll_get_counter_value_low(&SYSTIMER, PWM_ST_UNIT);
    uint32_t hi = systimer_ll_get_counter_value_high(&SYSTIMER, PWM_ST_UNIT);
    return ((uint64_t)hi << 32) | lo;
}

// Low 32 bits wrap every ~268 s at 16 MHz; every comparison against it is a
// difference, so wraparound is harmless.
static inline uint32_t IRAM_ATTR pwm_tmr_now(void) {
    return (uint32_t)pwm_tmr_now64();
}

static inline uint32_t IRAM_ATTR pwm_tmr_arm_rebase(uint32_t target, uint32_t ahead) {
    uint64_t now64 = pwm_tmr_now64();
    uint32_t now   = (uint32_t)now64;
    int32_t  wake  = (int32_t)(target - now);
    if (wake < (int32_t)(PWM_TMR_MIN_WAKE + ahead)) {
        wake = (int32_t)(PWM_TMR_MIN_WAKE + ahead);
    }
    // ORDER IS LOAD-BEARING. This alarm is one-shot, and in one-shot mode the
    // comparison "counter >= target" stays TRUE once the counter passes it. So
    // clearing the interrupt while the OLD, now-past target is still programmed
    // lets it re-latch immediately: the ISR re-enters, applies the next edge
    // early, and the following re-arm clamps to the minimum wake -- which
    // re-latches again. That is a self-sustaining interrupt storm at priority 4,
    // and since priority 4 preempts everything (watchdog included) the chip locks
    // solid and never reaches app code.
    //
    // Disable -> program a FUTURE target -> clear the stale latch -> re-enable.
    systimer_ll_enable_alarm(&SYSTIMER, PWM_ST_ALARM, false);
    systimer_ll_set_alarm_target(&SYSTIMER, PWM_ST_ALARM,
                                 now64 + (uint64_t)((uint32_t)wake - ahead));
    systimer_ll_apply_alarm_value(&SYSTIMER, PWM_ST_ALARM);
    systimer_ll_clear_alarm_int(&SYSTIMER, PWM_ST_ALARM);
    systimer_ll_enable_alarm(&SYSTIMER, PWM_ST_ALARM, true);
    return now + (uint32_t)wake;   // epoch never resets on this backend
}

static inline void IRAM_ATTR pwm_gpio_set(uint32_t mask) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask);
}
static inline void IRAM_ATTR pwm_gpio_clr(uint32_t mask) {
    REG_WRITE(GPIO_OUT_W1TC_REG, mask);
}

#endif  // BULB_PWM_TIMER_SYSTIMER_H
