// mono_clock.h — a monotonic CPU-cycle clock on the ESP8266 RTOS SDK.
//
// Why this exists: raw soc_get_ccount() is NOT free-running on this SDK. The RTOS
// tick handler (freertos/port/esp8266/port.c, xPortSysTickHandle) zeroes CCOUNT every
// tick (~10 ms) and accumulates the elapsed cycles into g_esp_os_cpu_clk — done to
// bound CCOUNT overflow and re-calibrate system time across long interrupt-off
// stretches (flash erase/write). So CCOUNT sawtooths 0..(one tick) and is useless as
// an absolute timebase, but g_esp_os_cpu_clk + soc_get_ccount() IS a true monotonic
// cycle count.
//
// The catch: a reader at NMI level (e.g. the PWM edge ISR, which rides the Wi-Fi TSF
// timer) can preempt the level-1 tick handler mid-update, between its "+= ccount" and
// "CCOUNT = 0" — where the accumulator already holds the cycles but CCOUNT is stale.
// To read consistently, mono_clock_start() takes over the tick ISR at runtime (no SDK
// fork — it just overwrites the ISR-table slot) with a copy that brackets the
// accumulate+reset in a generation counter; mono_ccount() uses it to pick the right
// formula. Because an NMI freezes the tick handler it preempts, a single snapshot is
// self-consistent — no seqlock spin (which would deadlock the NMI).

#ifndef BULB_MONO_CLOCK_H
#define BULB_MONO_CLOCK_H

#include <stdint.h>

#include "driver/soc.h"  // soc_get_ccount()

#ifdef __cplusplus
extern "C" {
#endif

// SDK global: running total of CPU cycles, advanced by the tick handler each tick.
extern uint64_t g_esp_os_cpu_clk;

// Odd while our tick handler is mid accumulate+reset (defined in mono_clock.c).
extern volatile uint32_t g_mono_ccount_seq;

// A monotonic 32-bit CPU-cycle count (low bits of the 64-bit total; use signed-diff
// math, which handles the clean 2^32 wrap). Safe to call from an NMI-level ISR: if we
// caught the tick handler mid-update (seq odd), the accumulator already holds the
// current cycles and CCOUNT is stale, so use the accumulator alone. Reads only DRAM +
// CCOUNT, so it is OTA-safe when inlined into an IRAM ISR.
static inline uint32_t mono_ccount(void) {
    const uint32_t seq = g_mono_ccount_seq;
    const uint32_t lo  = soc_get_ccount();
    const uint32_t acc = (uint32_t)g_esp_os_cpu_clk;
    return (seq & 1u) ? acc : (uint32_t)(acc + lo);
}

// Take over the RTOS tick ISR with the seq-bracketed copy so mono_ccount() is exact.
// Call exactly once, after the scheduler is running.
void mono_clock_start(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_MONO_CLOCK_H
