// settings.c — persisted device configuration in a dedicated flash sector.
//
// Layout of the sector (4 KB, only the head is used):
//   offset 0  : u32 magic      (RCSW_SET_MAGIC)
//   offset 4  : u32 version     (RCSW_SET_VERSION)
//   offset 8  : rcsw_settings_t (packed fields)
//   offset .. : u32 crc32       (over magic..end-of-struct)
//
// Writes go through the same erase/write recipe dfu2.c uses for the OTA
// partition (flash_ctrl WRITE_ENABLE / ERASE_SECTOR + flash_write, wrapped in
// flash_protection_op unprotect/reprotect). We only ever touch this one sector.

#include <stdint.h>
#include <string.h>

#include "flash_pub.h"   // CMD_FLASH_*, FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_PROTECT_*
#include "uart_pub.h"    // bk_printf

#include "config.h"
#include "settings.h"

// Dedicated user-config sector at the very TOP of flash (last 4 KB), verified
// against the SDK's 2M partition table (BkDriverFlash.c):
//   OTA        0x132000..0x1E0000   (DFU2 writes here; DFU2_OTA_LIMIT=0x1E0000)
//   RF_FIRMWARE0x1E0000..0x1E1000   (factory RF cal + MAC — never touch)
//   NET_PARAM  0x1E1000..0x1E2000   (live wifi params — never touch)
//   BLE_BONDING0x1F0000..0x1F1000   (unused: we run no BLE)
// 0x1FF000 is above ALL of these, unmapped, and OUTSIDE the DFU2 write range,
// so a firmware update can never clobber it. It's also above the end of our
// wired flash image (which stops at 0x132000), so config survives reflashes.
#ifndef RCSW_SETTINGS_ADDR
#define RCSW_SETTINGS_ADDR   0x1FF000u
#endif
#define RCSW_SET_SECTOR_SIZE 0x1000u
#define RCSW_SET_MAGIC       0x31534352u   // "RCS1"
#define RCSW_SET_VERSION     0x00000001u

extern void flash_init(void);
extern unsigned int flash_read(char *buf, unsigned int count, unsigned int addr);
extern unsigned int flash_write(char *buf, unsigned int count, unsigned int addr);
extern unsigned int flash_ctrl(unsigned int cmd, void *parm);
extern void flash_protection_op(unsigned char mode, unsigned char type);

// ---- CRC32 (IEEE 802.3, reflected) -----------------------------------------
static uint32_t crc32(const uint8_t *p, uint32_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1)));
    }
    return c ^ 0xFFFFFFFFu;
}

// On-flash record. This EMBEDS the in-RAM struct, so the flash layout is only
// stable while rcsw_settings_t stays padding-free (it is: 4x u8 then a u32,
// all naturally aligned). Padding bytes would be covered by the CRC and the
// save-verify memcmp but be indeterminate through struct assignment — if you
// add a field, keep the struct packed-by-construction, bump RCSW_SET_VERSION,
// and update the size pin below.
typedef struct {
    uint32_t magic;
    uint32_t version;
    rcsw_settings_t s;
    uint32_t crc;
} rcsw_record_t;

// Compile-time size pin (C89-compatible static assert).
typedef char rcsw_record_size_pin[(sizeof(rcsw_record_t) == 20) ? 1 : -1];

static void defaults(rcsw_settings_t *out) {
    out->my_id         = RCSW_MY_ID;
    out->wifi_channel  = RCSW_WIFI_CHANNEL;
    out->default_relay = RCSW_DEFAULT_RELAY;
    out->default_led   = RCSW_DEFAULT_LED;
    out->fallback_ms   = RCSW_FALLBACK_MS;
}

void rcsw_settings_load(rcsw_settings_t *out) {
    rcsw_record_t r;
    flash_init();
    flash_read((char *)&r, sizeof(r), RCSW_SETTINGS_ADDR);
    if (r.magic == RCSW_SET_MAGIC && r.version == RCSW_SET_VERSION &&
        r.crc == crc32((const uint8_t *)&r, sizeof(r) - sizeof(r.crc))) {
        *out = r.s;
        bk_printf("settings: loaded id=%d ch=%d dr=%d dl=%d fb=%u\r\n",
                  out->my_id, out->wifi_channel, out->default_relay,
                  out->default_led, (unsigned)out->fallback_ms);
        return;
    }
    defaults(out);
    bk_printf("settings: defaults (flash blank/invalid)\r\n");
}

bool rcsw_settings_save(const rcsw_settings_t *s) {
    static uint8_t page[RCSW_SET_SECTOR_SIZE];
    rcsw_record_t *r = (rcsw_record_t *)page;
    memset(page, 0xFF, sizeof(page));
    r->magic = RCSW_SET_MAGIC;
    r->version = RCSW_SET_VERSION;
    r->s = *s;
    r->crc = crc32((const uint8_t *)r, sizeof(*r) - sizeof(r->crc));

    unsigned int addr = RCSW_SETTINGS_ADDR;
    flash_init();
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_PROTECT_NONE);
    flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0);
    flash_ctrl(CMD_FLASH_ERASE_SECTOR, &addr);
    flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0);
    flash_write((char *)page, RCSW_SET_SECTOR_SIZE, addr);
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_UNPROTECT_LAST_BLOCK);

    // read-back verify (head only)
    rcsw_record_t chk;
    flash_read((char *)&chk, sizeof(chk), RCSW_SETTINGS_ADDR);
    bool ok = (chk.magic == RCSW_SET_MAGIC &&
               chk.crc == crc32((const uint8_t *)&chk, sizeof(chk) - sizeof(chk.crc)) &&
               memcmp(&chk.s, s, sizeof(*s)) == 0);
    bk_printf("settings: save %s\r\n", ok ? "OK" : "FAILED");
    return ok;
}

// ---- SetConfig application --------------------------------------------------
static uint32_t rd_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool rcsw_settings_apply(rcsw_settings_t *s, uint16_t key,
                         const uint8_t *val, uint8_t len) {
    switch (key) {
        case RCSW_CFG_KEY_MY_ID:
            if (len != 1) return false;
            if (s->my_id == val[0]) return false;
            s->my_id = val[0];
            return true;
        case RCSW_CFG_KEY_WIFI_CHANNEL:
            if (len != 1 || val[0] < 1 || val[0] > 14) return false;
            if (s->wifi_channel == val[0]) return false;
            s->wifi_channel = val[0];
            return true;
        case RCSW_CFG_KEY_DEFAULT_RELAY:
            if (len != 1 || val[0] > 1) return false;
            if (s->default_relay == val[0]) return false;
            s->default_relay = val[0];
            return true;
        case RCSW_CFG_KEY_DEFAULT_LED:
            if (len != 1 || val[0] > 1) return false;
            if (s->default_led == val[0]) return false;
            s->default_led = val[0];
            return true;
        case RCSW_CFG_KEY_FALLBACK_MS: {
            if (len != 4) return false;
            uint32_t v = rd_u32le(val);
            if (s->fallback_ms == v) return false;
            s->fallback_ms = v;
            return true;
        }
        default:
            return false;  // unknown key: ignore
    }
}
