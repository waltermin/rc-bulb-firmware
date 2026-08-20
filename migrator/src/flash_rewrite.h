#pragma once
#include <stdint.h>

// Rewrite low flash [0 .. sector_count*4096) from the payload at `src`
// (flash offset 0x111000), then software-reset into the RTOS bootloader.
// Runs with the flash cache disabled and interrupts masked: it and everything
// it calls are IRAM-resident and touch no flash-mapped (irom/PROGMEM) data.
// Never returns (resets or halts). Bootloader sectors 0/1/2 are written last.
void migrator_do_migration(uint32_t src, uint32_t sector_count);
