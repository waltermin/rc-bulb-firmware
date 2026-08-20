// ROM SPI-flash primitives, resolved by the core's eagle.rom.addr linker script.
// These live in mask ROM (not flash), so they are safe to call with the flash
// cache disabled during the migration write. Signatures mirror the eboot
// bootloader's bootloaders/eboot/flash.h.
#pragma once
#include <stdint.h>
#include <stddef.h>

extern "C" {
  // ROM primitives — safe with the flash cache DISABLED (as eboot uses them).
  int SPIRead(uint32_t addr, void *dst, size_t size);
  int SPIWrite(uint32_t addr, void *src, size_t size);
  int SPIEraseSector(uint32_t sector);
  void ets_wdt_disable(void);   // ROM; same one eboot uses during its copy
  void ets_wdt_enable(void);    // ROM; re-arm the hardware watchdog to force a full reset
}
// For the cache-on validation phase use the SDK's spi_flash_read (declared in
// <spi_flash.h>): it is cache-coordinated and reads the FULL chip correctly,
// unlike ROM SPIRead which mis-reads >1 MB with the cache enabled.
