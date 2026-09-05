// dfu2.h — pull-based OTA client for the BK7238 relay switch.
//
// Speaks the SAME DFU2 wire protocol as rc-bulb-firmware (0x05 Dfu2Request
// beacon -> join AP -> TCP pull from the update server -> hello/header/image +
// SHA-256 + build-id), but writes the image to the BK7238 rt_ota download
// partition (0x132000) instead of an ESP dual-OTA slot; the v1.0.15 bootloader
// verifies and swaps it into the app slot on reboot.

#ifndef RCSW_DFU2_H
#define RCSW_DFU2_H

#include <stdint.h>
#include "protocol.h"   // PROTO_DFU2_BUILD_ID_LEN

typedef struct {
    uint8_t  build_id[PROTO_DFU2_BUILD_ID_LEN];
    uint8_t  build_id_len;
    char     ssid[33];
    char     pass[64];
    uint8_t  server_ip[4];
    uint16_t server_port;
} dfu2_params_t;

// This build's id = sha256(version)[:PROTO_DFU2_BUILD_ID_LEN].
void dfu2_own_build_id(uint8_t out[PROTO_DFU2_BUILD_ID_LEN]);

// True while a DFU2 update is in progress (the main loop blinks the LED on it).
int dfu2_in_progress(void);

// Start the pull-based update on a dedicated task (returns immediately).
void dfu2_start(uint8_t my_id, const dfu2_params_t *params);

#endif  // RCSW_DFU2_H
