// dfu2srv.h — soft AP + DFU2 TCP server + 0x05 Dfu2Request beacons (see dfu2srv.c).

#ifndef DFU2SRV_H
#define DFU2SRV_H

#include <stdbool.h>
#include <stdint.h>

// Where the .rbl to serve lives in this S3's flash (outside the partition
// table; written with `esptool write-flash 0x800000 <image.rbl>`).
#define DFU2SRV_IMG_ADDR 0x800000u

typedef struct {
    uint32_t img_len;      // 0 = probe mode (ping each target, never send an image)
    uint8_t  build_id[8];  // sha256(version)[:8] of the served image
    uint8_t  dev_type;     // Dfu2Request device_type (0x02 = relay switch)
    uint8_t  id_start, id_bounds;
    const char *ssid, *pass;
    uint8_t  channel;      // AP + beacon channel (0 = 11); must match the targets'
} dfu2srv_params_t;

typedef void (*dfu2srv_log_fn)(const char *fmt, ...);

bool dfu2srv_start(const dfu2srv_params_t *p, dfu2srv_log_fn log);
bool dfu2srv_running(void);
void dfu2srv_target(uint8_t id_start, uint8_t id_bounds);  // retarget the beacons
void dfu2srv_stop(void);

#endif  // DFU2SRV_H
