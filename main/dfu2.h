// dfu2.h — pull-based DFU: on a 0x05 Dfu2Request the bulb joins the advertised
// AP, connects OUT to the update server as a TCP client, pulls a new firmware
// image into the inactive OTA slot, verifies it, and reboots.
//
// Reliability model (unchanged from the old push DFU): the running slot is never
// written during download, the image is verified (transport SHA-256 + esp_ota_end
// + a build-id re-check) before the boot pointer is flipped, and any failure
// reboots into the untouched current firmware. An optional NVS boot-counter
// rollback guard reverts a bad-but-bootable image.

#ifndef BULB_DFU2_H
#define BULB_DFU2_H

#include <stdint.h>

#include "protocol.h"  // PROTO_DFU2_* size constants

#ifdef __cplusplus
extern "C" {
#endif

// A self-contained copy of the fields from a Dfu2Request (the parser's pointers
// alias the RX buffer, so the caller copies them into this before handoff).
typedef struct {
    char     ssid[PROTO_DFU2_SSID_MAX + 1];         // NUL-terminated
    char     pass[PROTO_DFU2_PASS_MAX + 1];         // NUL-terminated ("" = open)
    uint8_t  build_id[PROTO_DFU2_BUILD_ID_LEN];     // target ELF-SHA256 prefix
    uint8_t  build_id_len;
    uint8_t  server_ip[4];                          // dotted-order octets
    uint16_t server_port;
} dfu2_params_t;

// Enter DFU2 mode. Spawns a dedicated task (large stack for wifi + lwIP + OTA +
// SHA) that stops the sniffer, joins the AP, pulls the image, and reboots.
// Returns immediately; the light path does not survive afterward. `params` is
// copied, so the caller need not keep it alive.
void dfu2_start(uint8_t my_id, const dfu2_params_t *params);

// This image's build id: SHA-256(esp_app_desc.version)[:PROTO_DFU2_BUILD_ID_LEN].
// The sniffer compares it against an advertised Dfu2Request build id to decide
// whether this bulb is out of date. Cached after the first call.
void dfu2_own_build_id(uint8_t out[PROTO_DFU2_BUILD_ID_LEN]);

// ---- optional rollback guard (see config.h DFU_ROLLBACK_GUARD) --------------
#include "config.h"
#ifdef DFU_ROLLBACK_GUARD

// Call early in app_main (after nvs_flash_init). If a freshly-flashed image has
// failed to validate across too many boots, this points the bootloader back at
// the previous slot and reboots. Otherwise it counts this boot.
void dfu2_rollback_check_on_boot(void);

// Call once the firmware has reached a known-good running state. Spawns a
// one-shot task that, after DFU_BOOT_VALIDATE_MS of continued running, marks the
// current image valid so it is no longer subject to rollback.
void dfu2_rollback_arm_validation(void);

#endif  // DFU_ROLLBACK_GUARD

#endif  // BULB_DFU2_H
