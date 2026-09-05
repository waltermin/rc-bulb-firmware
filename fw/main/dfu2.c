// dfu2.c — pull-based OTA client for the BK7238 relay switch.
//
// Same DFU2 wire protocol as rc-bulb-firmware (dfu2.c), BK7238 backend:
//   1. stop monitor mode, flag "in DFU" (main loop fast-blinks the LED)
//   2. join the AP named in the Dfu2Request (STA + DHCP)
//   3. TCP-connect OUT to the update server, retrying within a window
//   4. send hello (magic | fmt | my_id | own_build_id)
//   5. read header (magic | ver | image_len | sha256 | served_build_id), then
//      stream the image (an rt_ota RBL) into the download partition (0x132000),
//      hashing as we go
//   6. verify transport SHA-256 and served build-id == advertised target
//   7. best-effort status byte, reboot -> the v1.0.15 bootloader verifies the
//      RBL and swaps it into the app slot
//
// Reliability: the running app slot is never written during download (the RBL
// lands in the separate download partition), so a power loss mid-pull just
// reboots the current firmware, which re-triggers on the next broadcast. The
// only commit point is the bootloader's verified swap on reboot.

#include <stdint.h>
#include <string.h>

#include "rtos_pub.h"
#include "wlan_ui_pub.h"
#include "flash_pub.h"
#include "uart_pub.h"
#include "lwip/sockets.h"
#include "mbedtls/sha256.h"

#include "config.h"
#include "protocol.h"
#include "dfu2.h"

#ifndef RCSW_VERSION
#define RCSW_VERSION "rc_bare"
#endif

// BK7238 rt_ota download partition (OpenBeken's proven value for this chip).
#define DFU2_OTA_ADDR   0x132000u
#define DFU2_OTA_LIMIT  0x1E0000u   // stay below the config/KV region

#define DFU2_CONNECT_TIMEOUT_MS 30000   // AP join + DHCP
#define DFU2_TCP_WINDOW_MS      30000   // TCP connect retry window
#define DFU2_RECV_TIMEOUT_MS    15000   // per-recv stall guard

// wire constants (shared with rc-bulb-firmware / dfu2_server.py)
#define DFU2_HELLO_MAGIC 0x32756C52u   // "Rlu2"
#define DFU2_HDR_MAGIC   0x686C4352u   // "RClh"
#define DFU2_HELLO_FMT   0x01
#define DFU2_HDR_VERSION 0x01
#define DFU2_HELLO_LEN   (4 + 1 + 1 + PROTO_DFU2_BUILD_ID_LEN)   // 14
#define DFU2_HDR_LEN     (4 + 1 + 4 + 32 + PROTO_DFU2_BUILD_ID_LEN)  // 49

enum { DFU_ST_OK=0, DFU_ST_BAD_HEADER=1, DFU_ST_OTA_BEGIN=2, DFU_ST_RECV=3,
       DFU_ST_SHA=4, DFU_ST_OTA_END=5, DFU_ST_SET_BOOT=6, DFU_ST_BUILD_ID=7, DFU_ST_NET=8 };

extern void flash_init(void);
extern unsigned int flash_read(char *buf, unsigned int count, unsigned int addr);
extern unsigned int flash_write(char *buf, unsigned int count, unsigned int addr);
extern unsigned int flash_ctrl(unsigned int cmd, void *parm);
extern void flash_protection_op(unsigned char mode, unsigned char type);
extern void bk_reboot(void);

static volatile int g_dfu_active = 0;
int dfu2_in_progress(void) { return g_dfu_active; }

// ---- build id --------------------------------------------------------------
void dfu2_own_build_id(uint8_t out[PROTO_DFU2_BUILD_ID_LEN]) {
    static uint8_t cached[PROTO_DFU2_BUILD_ID_LEN];
    static int have = 0;
    if (!have) {
        uint8_t digest[32];
        mbedtls_sha256_ret((const unsigned char *)RCSW_VERSION, strlen(RCSW_VERSION), digest, 0);
        memcpy(cached, digest, PROTO_DFU2_BUILD_ID_LEN);
        have = 1;
    }
    memcpy(out, cached, PROTO_DFU2_BUILD_ID_LEN);
}

// ---- download-partition writer (ported from OpenBeken hal_ota_bk7231.c) -----
#define SECTOR_SIZE 0x1000u
static uint8_t *g_sector;
static unsigned int g_sectorlen;
static unsigned int g_addr;

static void store_sector(unsigned int addr, uint8_t *data) {
    flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0);
    flash_ctrl(CMD_FLASH_ERASE_SECTOR, &addr);
    flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0);
    flash_write((char *)data, SECTOR_SIZE, addr);
}

static int ota_init(unsigned int startaddr) {
    flash_init();
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_PROTECT_NONE);
    g_sector = os_malloc(SECTOR_SIZE);
    if (!g_sector) return 0;
    g_sectorlen = 0;
    g_addr = startaddr;
    return 1;
}

static void ota_add(const uint8_t *data, int len) {
    while (len > 0) {
        unsigned int room = SECTOR_SIZE - g_sectorlen;
        unsigned int n = ((unsigned)len < room) ? (unsigned)len : room;
        memcpy(g_sector + g_sectorlen, data, n);
        data += n; len -= (int)n; g_sectorlen += n;
        if (g_sectorlen == SECTOR_SIZE) {
            store_sector(g_addr, g_sector);
            g_addr += SECTOR_SIZE;
            g_sectorlen = 0;
        }
    }
}

static void ota_close(void) {
    if (g_sectorlen) {
        memset(g_sector + g_sectorlen, 0xFF, SECTOR_SIZE - g_sectorlen);
        store_sector(g_addr, g_sector);
        g_addr += SECTOR_SIZE;
        g_sectorlen = 0;
    }
    if (g_sector) { os_free(g_sector); g_sector = (void *)0; }
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_UNPROTECT_LAST_BLOCK);
}

// ---- network ----------------------------------------------------------------
static int join_ap(const dfu2_params_t *p) {
    network_InitTypeDef_st cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.wifi_mode = BK_STATION;
    strncpy(cfg.wifi_ssid, p->ssid, sizeof(cfg.wifi_ssid) - 1);
    strncpy(cfg.wifi_key, p->pass, sizeof(cfg.wifi_key) - 1);
    cfg.dhcp_mode = DHCP_CLIENT;
    cfg.wifi_retry_interval = 100;
    bk_wlan_start(&cfg);

    // Poll for a DHCP lease.
    for (int i = 0; i < (DFU2_CONNECT_TIMEOUT_MS / 200); i++) {
        rtos_delay_milliseconds(200);
        IPStatusTypedef ip;
        memset(&ip, 0, sizeof(ip));
        bk_wlan_get_ip_status(&ip, BK_STATION);
        if (ip.ip[0] && strcmp(ip.ip, "0.0.0.0") != 0) {
            bk_printf("dfu2: got ip %s\r\n", ip.ip);
            return 1;
        }
    }
    return 0;
}

static int connect_server(const dfu2_params_t *p) {
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(p->server_port);
    uint32_t a = ((uint32_t)p->server_ip[0]) | ((uint32_t)p->server_ip[1] << 8) |
                 ((uint32_t)p->server_ip[2] << 16) | ((uint32_t)p->server_ip[3] << 24);
    dst.sin_addr.s_addr = a;

    for (int i = 0; i < (DFU2_TCP_WINDOW_MS / 1000); i++) {
        int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s >= 0) {
            if (connect(s, (struct sockaddr *)&dst, sizeof(dst)) == 0) return s;
            close(s);
        }
        rtos_delay_milliseconds(1000);
    }
    return -1;
}

static int recv_exact(int s, uint8_t *buf, int n) {
    int got = 0;
    while (got < n) {
        int r = recv(s, buf + got, n - got, 0);
        if (r <= 0) return 0;
        got += r;
    }
    return 1;
}

static int run_pull(uint8_t my_id, const dfu2_params_t *p) {
    int s = connect_server(p);
    if (s < 0) return DFU_ST_NET;

    struct timeval rto = { .tv_sec = DFU2_RECV_TIMEOUT_MS / 1000, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof(rto));

    uint8_t hello[DFU2_HELLO_LEN];
    hello[0] = (uint8_t)DFU2_HELLO_MAGIC;       hello[1] = (uint8_t)(DFU2_HELLO_MAGIC >> 8);
    hello[2] = (uint8_t)(DFU2_HELLO_MAGIC >> 16); hello[3] = (uint8_t)(DFU2_HELLO_MAGIC >> 24);
    hello[4] = DFU2_HELLO_FMT;
    hello[5] = my_id;
    dfu2_own_build_id(&hello[6]);
    if (send(s, hello, sizeof(hello), 0) != (int)sizeof(hello)) { close(s); return DFU_ST_NET; }

    int status = DFU_ST_BAD_HEADER;
    uint8_t hdr[DFU2_HDR_LEN];
    if (recv_exact(s, hdr, sizeof(hdr))) {
        uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        uint8_t ver = hdr[4];
        uint32_t image_len = (uint32_t)hdr[5] | ((uint32_t)hdr[6] << 8) | ((uint32_t)hdr[7] << 16) | ((uint32_t)hdr[8] << 24);
        const uint8_t *sha = &hdr[9];
        const uint8_t *served_id = &hdr[9 + 32];

        if (magic != DFU2_HDR_MAGIC || ver != DFU2_HDR_VERSION || image_len == 0 ||
            (DFU2_OTA_ADDR + image_len) > DFU2_OTA_LIMIT) {
            status = DFU_ST_BAD_HEADER;
        } else if (memcmp(served_id, p->build_id, p->build_id_len) != 0) {
            status = DFU_ST_BUILD_ID;
        } else if (!ota_init(DFU2_OTA_ADDR)) {
            status = DFU_ST_OTA_BEGIN;
        } else {
            bk_printf("dfu2: receiving %u bytes -> 0x%x\r\n", (unsigned)image_len, DFU2_OTA_ADDR);
            mbedtls_sha256_context sh;
            mbedtls_sha256_init(&sh);
            mbedtls_sha256_starts_ret(&sh, 0);
            static uint8_t buf[1024];
            uint32_t rem = image_len;
            status = DFU_ST_OK;
            while (rem > 0) {
                int want = rem < sizeof(buf) ? (int)rem : (int)sizeof(buf);
                int r = recv(s, buf, want, 0);
                if (r <= 0) { status = DFU_ST_RECV; break; }
                ota_add(buf, r);
                mbedtls_sha256_update_ret(&sh, buf, r);
                rem -= (uint32_t)r;
            }
            uint8_t digest[32];
            mbedtls_sha256_finish_ret(&sh, digest);
            mbedtls_sha256_free(&sh);
            ota_close();   // flush + re-protect regardless
            if (status == DFU_ST_OK && memcmp(digest, sha, 32) != 0) status = DFU_ST_SHA;
        }
    }

    uint8_t st = (uint8_t)status;
    send(s, &st, 1, 0);
    shutdown(s, SHUT_RDWR);
    close(s);
    return status;
}

// ---- task -------------------------------------------------------------------
typedef struct { uint8_t my_id; dfu2_params_t p; } dfu2_job_t;

static void dfu2_task(void *arg) {
    dfu2_job_t *job = (dfu2_job_t *)arg;
    g_dfu_active = 1;
    bk_printf("dfu2: entering (id %u, ap '%s')\r\n", job->my_id, job->p.ssid);

    bk_wlan_stop_monitor();
    rtos_delay_milliseconds(200);

    int status = DFU_ST_NET;
    if (join_ap(&job->p)) {
        status = run_pull(job->my_id, &job->p);
    } else {
        bk_printf("dfu2: could not join AP\r\n");
    }
    bk_printf("dfu2: finished status=%d; rebooting\r\n", status);
    rtos_delay_milliseconds(800);
    bk_reboot();
    // (bk_reboot does not return; if it somehow does, drop the flag)
    g_dfu_active = 0;
    os_free(job);
    rtos_delete_thread(NULL);
}

void dfu2_start(uint8_t my_id, const dfu2_params_t *params) {
    dfu2_job_t *job = os_malloc(sizeof(*job));
    if (!job) { bk_reboot(); return; }
    job->my_id = my_id;
    memcpy(&job->p, params, sizeof(*params));
    beken_thread_t th;
    if (rtos_create_thread(&th, THD_APPLICATION_PRIORITY, "dfu2",
                           (beken_thread_function_t)dfu2_task, 8192,
                           (beken_thread_arg_t)job) != kNoErr) {
        os_free(job);
        bk_printf("dfu2: failed to start task\r\n");
    }
}
