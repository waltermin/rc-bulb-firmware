#include <Arduino.h>
#include <esp8266_undocumented.h>   // Wait_SPI_Idle, Cache_Read_Disable, flashchip, ets_delay_us
#include <ets_sys.h>               // ets_wdt_disable
#include "flash_rewrite.h"
#include "rom_funcs.h"
#include "leds.h"

// Everything here runs with the flash cache DISABLED: this function and every
// function it calls must be IRAM_ATTR and must not read flash-mapped
// (irom/PROGMEM/const-global/string-literal) data. Only ROM funcs (SPIRead/
// SPIWrite/SPIEraseSector/ets_*/Wait_SPI_Idle/Cache_Read_Disable) and direct
// register writes are used. NOTE: analogWrite/startWaveform are NOT IRAM-safe,
// so LEDs here are driven by manual software PWM on the GPIO registers.

#define SECTOR      4096u

// Direct hardware registers (safe with the cache off; no function calls).
#define GPIO_OUT_SET (*(volatile uint32_t *)0x60000304)  // write 1<<pin -> drive high
#define GPIO_OUT_CLR (*(volatile uint32_t *)0x60000308)  // write 1<<pin -> drive low
#define TIMER1_CTRL  (*(volatile uint32_t *)0x60000608)  // FRC1 control; bit7 = enable

// Stop the Timer1 NMI that analogWrite() used for the blue PWM, then force all
// three diagnostic channels low, so a manual PWM below shows a clean single color.
static void IRAM_ATTR leds_all_off() {
  TIMER1_CTRL &= ~0x80u;   // disable Timer1 -> no more PWM NMI
  GPIO_OUT_CLR = (1u << LED_R_PIN) | (1u << LED_G_PIN) | (1u << LED_B_PIN);
}

// One manual-PWM period (LED_MAX duty) on `pin`.
static inline void IRAM_ATTR led_pwm_period(uint8_t pin) {
  GPIO_OUT_SET = (1u << pin); ets_delay_us(LED_ON_US);
  GPIO_OUT_CLR = (1u << pin); ets_delay_us(LED_OFF_US);
}

// SUCCESS: solid green, then let the hardware watchdog reset us into the new
// firmware. The RTC "SWRST" is only a CPU-level reset and, from our post-write
// state (cache off, ICACHE bits cleared), the ROM boot hangs afterward; a
// REASON_WDT_RST is a full system reset that re-inits the cache/SPI controller so
// the freshly written RTOS bootloader boots cleanly. Green shows until it bites.
static void IRAM_ATTR success_green_then_reset() {
  leds_all_off();
  ets_wdt_enable();
  for (;;) led_pwm_period(LED_G_PIN);
}

// FAILURE after we began writing: flash red forever (recover via serial). ~180 ms
// red at LED_MAX duty, ~180 ms off.
static void IRAM_ATTR fail_red_forever() {
  leds_all_off();
  const uint32_t on_periods = 180000u / LED_PERIOD_US;
  for (;;) {
    for (uint32_t i = 0; i < on_periods; i++) led_pwm_period(LED_R_PIN);
    GPIO_OUT_CLR = (1u << LED_R_PIN);
    ets_delay_us(180000);
  }
}

// DRAM (.bss) scratch — never on the stack (4 KB each) and never in flash.
static uint8_t s_buf[SECTOR]  __attribute__((aligned(4)));
static uint8_t s_buf2[SECTOR] __attribute__((aligned(4)));

// IRAM-resident 4 KB compare; returns 0 if equal, 1 if different.
static int IRAM_ATTR iram_diff(const uint32_t *a, const uint32_t *b) {
  for (uint32_t i = 0; i < SECTOR / 4; i++) {
    if (a[i] != b[i]) return 1;
  }
  return 0;
}

// Copy one sector src->dst with erase; returns 0 on success.
static int IRAM_ATTR write_sector(uint32_t src, uint32_t dst) {
  if (SPIRead(src, s_buf, SECTOR)) return 1;
  if (SPIEraseSector(dst / SECTOR)) return 1;
  if (SPIWrite(dst, s_buf, SECTOR)) return 1;
  return 0;
}

// Read `dst` back and compare to `src`; returns 0 if they match.
static int IRAM_ATTR verify_sector(uint32_t src, uint32_t dst) {
  if (SPIRead(src, s_buf,  SECTOR)) return 1;
  if (SPIRead(dst, s_buf2, SECTOR)) return 1;
  return iram_diff((const uint32_t *)s_buf, (const uint32_t *)s_buf2);
}

void IRAM_ATTR migrator_do_migration(uint32_t src, uint32_t sector_count) {
  // Enter the "touch flash while not executing from it" state (mirrors
  // cores/esp8266/reboot_uart_dwnld.cpp), minus the UART-download tail. The blue
  // "migrating" PWM set up by analogWrite() keeps running here: it is driven by
  // the Timer1 NMI (non-maskable, IRAM), so it survives the cache-off write.
  ets_wdt_disable();
  (void)xt_rsil(15);                 // mask all level-1 interrupts (also the soft WDT)
  Wait_SPI_Idle(flashchip);
  Cache_Read_Disable();
  CLEAR_PERI_REG_MASK(PERIPHS_DPORT_ICACHE_ENABLE,
                      ICACHE_ENABLE_FIRST_16K | ICACHE_ENABLE_SECOND_16K);

  const uint32_t BOOT_SECTORS = 3;   // 0x0,0x1000,0x2000 = the RTOS bootloader

  // PASS 1 — write everything except the bootloader sectors.
  for (uint32_t s = BOOT_SECTORS; s < sector_count; s++) {
    if (write_sector(src + s * SECTOR, s * SECTOR)) fail_red_forever();
  }

  // PASS 2 — verify pass 1 before committing the bootloader.
  for (uint32_t s = BOOT_SECTORS; s < sector_count; s++) {
    if (verify_sector(src + s * SECTOR, s * SECTOR)) fail_red_forever();
  }

  // PASS 3 — bootloader LAST, boot vector at sector 0 written last of all.
  // sectors 2, 1, 0 (unrolled: no flash-resident array in the critical section)
  if (write_sector(src + 2 * SECTOR, 2 * SECTOR)) fail_red_forever();
  if (verify_sector(src + 2 * SECTOR, 2 * SECTOR)) fail_red_forever();
  if (write_sector(src + 1 * SECTOR, 1 * SECTOR)) fail_red_forever();
  if (verify_sector(src + 1 * SECTOR, 1 * SECTOR)) fail_red_forever();
  if (write_sector(src + 0 * SECTOR, 0 * SECTOR)) fail_red_forever();
  if (verify_sector(src + 0 * SECTOR, 0 * SECTOR)) fail_red_forever();

  // Done: solid green, then a watchdog reset into the freshly written firmware.
  success_green_then_reset();
}
