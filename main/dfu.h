// dfu.h — DFU mode: leave promiscuous, join a hard-coded AP, and receive a new
// firmware image over TCP into the inactive OTA slot.

#ifndef BULB_DFU_H
#define BULB_DFU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Enter DFU mode. Spawns a dedicated task (with a large stack for the TCP +
// OTA + SHA work) that stops the sniffer, resets to the default color, connects
// to the configured AP (DHCP hostname rc_light_dfu_<my_id>), runs the
// firmware-push server, and reboots. Returns immediately; the caller should not
// assume the light path keeps running afterward.
void dfu_start(uint8_t my_id);

#ifdef __cplusplus
}
#endif

// ---- optional rollback guard (see config.h DFU_ROLLBACK_GUARD) --------------
#include "config.h"
#ifdef DFU_ROLLBACK_GUARD

// Call early in app_main (after nvs_flash_init). If a freshly-flashed image has
// failed to validate across too many boots, this points the bootloader back at
// the previous slot and reboots. Otherwise it counts this boot.
void dfu_rollback_check_on_boot(void);

// Call once the firmware has reached a known-good running state. Spawns a
// one-shot task that, after DFU_BOOT_VALIDATE_MS of continued running, marks the
// current image valid so it is no longer subject to rollback.
void dfu_rollback_arm_validation(void);

#endif  // DFU_ROLLBACK_GUARD

#endif  // BULB_DFU_H
