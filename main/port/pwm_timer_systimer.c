// pwm_timer_systimer.c -- ESP32-C3 timer backend setup.

#include "pwm_timer_systimer.h"

#include "esp_intr_alloc.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/interrupts.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "pwm_tmr";

// Serialises task-level producers only; see the note in pwm_timer.h.
portMUX_TYPE g_pwm_mux = portMUX_INITIALIZER_UNLOCKED;

static void (*s_engine_isr)(void);
static intr_handle_t s_handle;

// esp_intr_alloc hands the handler a void* the engine ISR does not want.
static void IRAM_ATTR pwm_tmr_trampoline(void *arg) {
    (void)arg;
    s_engine_isr();
}

void pwm_tmr_init(void (*isr)(void)) {
    s_engine_isr = isr;

    // One-shot against the shared counter: the engine computes a fresh absolute
    // target after every edge, so periodic mode would only fight it.
    systimer_ll_connect_alarm_counter(&SYSTIMER, PWM_ST_ALARM, PWM_ST_UNIT);
    systimer_ll_enable_alarm(&SYSTIMER, PWM_ST_ALARM, false);
    systimer_ll_enable_alarm_oneshot(&SYSTIMER, PWM_ST_ALARM);
    systimer_ll_clear_alarm_int(&SYSTIMER, PWM_ST_ALARM);

    // esp_intr_alloc rejects any level above 3 on RISC-V with ESP_ERR_INVALID_ARG
    // -- it refuses before it even searches. But the INTC priority register takes
    // 1..15, and critical sections only raise the threshold to 3. So: allocate
    // legally at 3, then reprogram the hardware priority to 4. Both calls are
    // public API (esp_intr_alloc.h, esp_cpu.h); nothing private is used, which is
    // a real improvement on the ESP8266's undocumented wDev_MacTimSetFunc.
    //
    // Consequence, and it is strict: this ISR runs INSIDE other code's critical
    // sections. It must never call a FreeRTOS API (not even the FromISR forms),
    // never take a lock, and touch only IRAM/DRAM -- it can fire with the flash
    // cache disabled. The engine ISR already obeys all three.
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_SYSTIMER_TARGET1_INTR_SOURCE,
                                   ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                                   pwm_tmr_trampoline, NULL, &s_handle));
    int intno = esp_intr_get_intno(s_handle);
    esp_cpu_intr_set_priority(intno, PWM_INTR_PRIORITY);

    ESP_LOGI(TAG, "systimer TARGET1 -> cpu_int %d at priority %d (above critical sections)",
             intno, PWM_INTR_PRIORITY);
}

void pwm_tmr_enable(void) {
    systimer_ll_enable_alarm_int(&SYSTIMER, PWM_ST_ALARM, true);
}
