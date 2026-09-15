// pwm_timer_wdev.c -- ESP8266 timer backend setup (WDEV/TSF0 at NMI level).

#include "pwm_timer_wdev.h"
#include "esp8266/eagle_soc.h"

// Private Wi-Fi-library hook: registers a callback into the WDEV/TSF0 (Wi-Fi MAC)
// timer's compare interrupt -- the same high-priority timer the stock SDK PWM
// rides. NMI level is above the level-1 critical sections that made FRC1 flicker.
extern int wDev_MacTimSetFunc(void (*handle)(void));

void pwm_tmr_init(void (*isr)(void)) {
    wDev_MacTimSetFunc(isr);
}

// Enable only AFTER the first compare is armed at a fresh future value (the exact
// order the SDK PWM uses), so the first interrupt comes from our schedule rather
// than a stale compare with the target uninitialised.
void pwm_tmr_enable(void) {
    REG_WRITE(PERIPHS_DPORT_BASEADDR, (REG_READ(PERIPHS_DPORT_BASEADDR) & ~0x1F) | 0x1);
    REG_WRITE(INT_ENA_WDEV, REG_READ(INT_ENA_WDEV) | WDEV_TSF0_REACH_INT);
}
