// mono_clock.c — runtime-installed RTOS tick handler that makes the CPU-cycle count
// monotonically readable. See mono_clock.h for the rationale.
//
// mono_tick_handle() is a copy of the SDK's xPortSysTickHandle
// (components/freertos/port/esp8266/port.c) — byte-for-byte the same work — with the
// generation counter g_mono_ccount_seq bracketing the CCOUNT accumulate+reset. We
// install it over the SDK's handler at runtime (mono_clock_start), so the SDK source
// is untouched. It must stay in sync with the SDK's handler; that function is small
// and stable, and we build against a pinned SDK.

#include "mono_clock.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  // vTaskStepTick, xTaskIncrementTick

#include "driver/soc.h"     // soc_get_ccount / soc_set_ccount / soc_set_ccompare
#include "esp_attr.h"       // IRAM_ATTR
#include "rom/ets_sys.h"    // ETS_MAX_INUM (the RTOS tick interrupt slot)

// SDK globals the tick handler maintains (all non-static in port.c).
extern uint64_t g_esp_os_us;         // running total microseconds
extern uint64_t g_esp_os_ticks;      // running total RTOS ticks
extern uint32_t g_esp_ticks_per_us;  // CPU cycles per microsecond
extern uint32_t _xt_tick_divisor;    // CPU cycles per RTOS tick

volatile uint32_t g_mono_ccount_seq;

static void IRAM_ATTR mono_tick_handle(void *p) {
    (void)p;
    const uint32_t ccount = soc_get_ccount();
    const uint32_t us = ccount / g_esp_ticks_per_us;
    g_esp_os_us += us;

    // Update the accumulator FIRST, then flip seq odd. That way "seq odd" always
    // means the accumulator is already current (CCOUNT is the stale part), so
    // mono_ccount()'s "seq odd -> use accumulator alone" is correct, and the only
    // unprotected window is the single seq++ store below (~1 cycle) rather than the
    // multi-instruction 64-bit add. Barriers stop the compiler reordering across seq.
    g_esp_os_cpu_clk += ccount;
    __asm__ volatile("" ::: "memory");
    g_mono_ccount_seq++;                 // odd: accumulator current, CCOUNT stale
    soc_set_ccount(0);
    soc_set_ccompare(_xt_tick_divisor);
    __asm__ volatile("" ::: "memory");
    g_mono_ccount_seq++;                 // even: CCOUNT reset, accumulator+CCOUNT valid

    const uint32_t ticks = us / 1000 / portTICK_PERIOD_MS;
    if (ticks > 1) {
        vTaskStepTick(ticks - 1);
    }
    g_esp_os_ticks++;
    if (xTaskIncrementTick() != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

void mono_clock_start(void) {
    // Overwrite the ISR-table slot the scheduler filled with xPortSysTickHandle. A
    // single pointer store; at worst one tick runs the old (still-correct) handler.
    _xt_isr_attach(ETS_MAX_INUM, mono_tick_handle, NULL);
}
