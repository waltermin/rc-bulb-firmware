// dfu.c — DFU mode + OTA writer.
//
// Reliability model (ESP8266_RTOS_SDK has no native app-rollback API):
//   * The running app's slot is NEVER written during download. A power loss at
//     any point leaves the current firmware intact and bootable.
//   * We stream into the INACTIVE slot, verify a transport SHA-256 AND let
//     esp_ota_end() validate the image, and only THEN flip the boot pointer.
//   * The boot-slot switch is a single atomic otadata (dual-copy) update.
//   * XIP safety is handled by the SDK: flash routines run from IRAM with the
//     cache + interrupts disabled per op. Our only other flash-time ISR (PWM)
//     is IRAM-resident, so LEDs keep running through the update.
//   * Optional NVS boot-counter guard reverts a bad-but-bootable image.

#include "dfu.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "tcpip_adapter.h"
#include "nvs.h"

#include "lwip/sockets.h"
#include "mbedtls/sha256.h"

#include "config.h"
#include "bulb_config.h"
#include "pwm_output.h"
#include "sniffer.h"

static const char *TAG = "dfu";

// ---- DFU push wire header ----------------------------------------------------
// magic u32 LE | version u8 | image_len u32 LE | sha256[32]   => 41 bytes
#define DFU_HDR_LEN 41
#define DFU_HDR_VERSION 0x01

// status bytes sent back to the pusher
enum {
    DFU_ST_OK = 0,
    DFU_ST_BAD_HEADER = 1,
    DFU_ST_OTA_BEGIN = 2,
    DFU_ST_RECV = 3,
    DFU_ST_SHA = 4,
    DFU_ST_OTA_END = 5,
    DFU_ST_SET_BOOT = 6,
};

// ---- wifi connect bookkeeping ----------------------------------------------
#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;
static char s_ip_str[16] = "0.0.0.0";  // acquired DHCP address, for logging

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        // Keep retrying until the outer timeout gives up.
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // Record + log the DHCP address so it can be found (esp. on the devboard
        // where we have serial; real bulbs are found via the rc_light_dfu_<id>
        // DHCP hostname instead). ip is network byte order; octet 1 is the low byte.
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        uint32_t a = evt->ip_info.ip.addr;
        snprintf(s_ip_str, sizeof(s_ip_str), "%u.%u.%u.%u",
                 (unsigned)(a & 0xff), (unsigned)((a >> 8) & 0xff),
                 (unsigned)((a >> 16) & 0xff), (unsigned)((a >> 24) & 0xff));
        ESP_LOGI(TAG, "DFU: got IP %s", s_ip_str);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static bool connect_to_ap(uint8_t my_id) {
    s_wifi_events = xEventGroupCreate();

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL);

    // Hostname must be set before the DHCP client starts (i.e. before connect).
    char host[32];
    snprintf(host, sizeof(host), "%s%u", DFU_HOSTNAME_PREFIX, (unsigned)my_id);
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, host);
    ESP_LOGI(TAG, "DFU hostname: %s", host);

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, bulb_config_get_str(CFG_DFU_SSID), sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, bulb_config_get_str(CFG_DFU_PASS), sizeof(wc.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wc));
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(DFU_CONNECT_TIMEOUT_MS));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

// ---- reliable recv of exactly n bytes --------------------------------------
static bool recv_exact(int sock, uint8_t *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        int r = recv(sock, buf + got, n - got, 0);
        if (r <= 0) {
            return false;
        }
        got += (size_t)r;
    }
    return true;
}

// ---- optional rollback guard: record the pending state before we reboot -----
static void rollback_mark_pending(void) {
#ifdef DFU_ROLLBACK_GUARD
    const esp_partition_t *running = esp_ota_get_running_partition();
    nvs_handle h;
    if (nvs_open("dfu", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "pending", 1);
        nvs_set_u8(h, "bootcnt", 0);
        // Remember the currently-running (known-good) slot to revert to.
        nvs_set_u8(h, "prev", running ? (uint8_t)running->subtype : 0);
        nvs_commit(h);
        nvs_close(h);
    }
#endif
}

// Receive the streamed image into `part`, verifying size + SHA-256. Returns a
// DFU_ST_* status. On DFU_ST_OK the caller may flip the boot partition.
static int receive_image(int sock, const esp_partition_t *part,
                         uint32_t image_len, const uint8_t expected_sha[32]) {
    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(part, image_len, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %d", err);
        return DFU_ST_OTA_BEGIN;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);  // 0 => SHA-256

    static uint8_t buf[1024];
    uint32_t remaining = image_len;
    while (remaining > 0) {
        size_t chunk = remaining < sizeof(buf) ? remaining : sizeof(buf);
        int r = recv(sock, buf, chunk, 0);
        if (r <= 0) {
            ESP_LOGE(TAG, "recv failed with %u bytes left", (unsigned)remaining);
            mbedtls_sha256_free(&sha);
            esp_ota_end(handle);
            return DFU_ST_RECV;
        }
        if (esp_ota_write(handle, buf, r) != ESP_OK) {
            mbedtls_sha256_free(&sha);
            esp_ota_end(handle);
            return DFU_ST_OTA_END;
        }
        mbedtls_sha256_update_ret(&sha, buf, r);
        remaining -= (uint32_t)r;
    }

    uint8_t digest[32];
    mbedtls_sha256_finish_ret(&sha, digest);
    mbedtls_sha256_free(&sha);

    if (memcmp(digest, expected_sha, 32) != 0) {
        ESP_LOGE(TAG, "SHA-256 mismatch; discarding image");
        esp_ota_end(handle);
        return DFU_ST_SHA;
    }

    // esp_ota_end validates the on-flash image (magic/checksum) before we trust it.
    if (esp_ota_end(handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end validation failed");
        return DFU_ST_OTA_END;
    }
    return DFU_ST_OK;
}

// Run the TCP push server: accept one client, read the header, receive+verify
// the image, and (on success) set the boot partition. Returns a DFU_ST_* code.
static int run_push_server(void) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        return DFU_ST_RECV;
    }
    int one = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(DFU_TCP_PORT);
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listen_sock, 1) != 0) {
        close(listen_sock);
        return DFU_ST_RECV;
    }
    ESP_LOGI(TAG, "DFU server listening on %s:%d  -->  push_dfu.py %s <fw.bin>",
             s_ip_str, DFU_TCP_PORT, s_ip_str);

    // Bounded wait for a client.
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(listen_sock, &rfds);
    struct timeval tv = {.tv_sec = DFU_ACCEPT_TIMEOUT_MS / 1000, .tv_usec = 0};
    if (select(listen_sock + 1, &rfds, NULL, NULL, &tv) <= 0) {
        ESP_LOGE(TAG, "no DFU client connected in time");
        close(listen_sock);
        return DFU_ST_RECV;
    }

    int sock = accept(listen_sock, NULL, NULL);
    close(listen_sock);
    if (sock < 0) {
        return DFU_ST_RECV;
    }

    // Guard against a stalled client mid-transfer.
    struct timeval rto = {.tv_sec = 15, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof(rto));

    int status = DFU_ST_BAD_HEADER;
    uint8_t hdr[DFU_HDR_LEN];
    if (recv_exact(sock, hdr, sizeof(hdr))) {
        uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                         ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        uint8_t version = hdr[4];
        uint32_t image_len = (uint32_t)hdr[5] | ((uint32_t)hdr[6] << 8) |
                             ((uint32_t)hdr[7] << 16) | ((uint32_t)hdr[8] << 24);
        const uint8_t *sha = &hdr[9];

        const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
        if (magic != DFU_OTA_MAGIC || version != DFU_HDR_VERSION ||
            image_len == 0 || part == NULL || image_len > part->size) {
            ESP_LOGE(TAG, "bad header (magic=%08x ver=%d len=%u)",
                     (unsigned)magic, version, (unsigned)image_len);
            status = DFU_ST_BAD_HEADER;
        } else {
            ESP_LOGI(TAG, "receiving %u bytes into %s", (unsigned)image_len, part->label);
            status = receive_image(sock, part, image_len, sha);
            if (status == DFU_ST_OK) {
                if (esp_ota_set_boot_partition(part) == ESP_OK) {
                    rollback_mark_pending();
                    ESP_LOGI(TAG, "image accepted; booting %s", part->label);
                } else {
                    status = DFU_ST_SET_BOOT;
                }
            }
        }
    }

    // Best-effort status reply, then make sure it flushes before we reboot.
    uint8_t st = (uint8_t)status;
    send(sock, &st, 1, 0);
    shutdown(sock, SHUT_RDWR);
    close(sock);
    return status;
}

static void dfu_task(void *arg) {
    uint8_t my_id = (uint8_t)(uintptr_t)arg;
    ESP_LOGW(TAG, "entering DFU mode");

    sniffer_stop();
    // Steady, known LED state during the update: 50% blue signals "in DFU mode".
    pwm_output_set(0.0f, 0.0f, 0.5f, 0.0f, 0.0f);

    if (!connect_to_ap(my_id)) {
        ESP_LOGE(TAG, "failed to join AP; rebooting");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    int status = run_push_server();
    ESP_LOGI(TAG, "DFU finished with status %d; rebooting", status);
    vTaskDelay(pdMS_TO_TICKS(500));  // let logs/reply flush
    esp_restart();
}

void dfu_start(uint8_t my_id) {
    // Dedicated task: the DFU path needs a much larger stack than the light
    // controller (wifi connect + lwIP sockets + esp_ota_write + mbedTLS SHA).
    xTaskCreate(dfu_task, "dfu", 8192, (void *)(uintptr_t)my_id, 6, NULL);
}

// ---- rollback guard implementation -----------------------------------------
#ifdef DFU_ROLLBACK_GUARD

static void rollback_validate_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(DFU_BOOT_VALIDATE_MS));
    nvs_handle h;
    if (nvs_open("dfu", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "pending", 0);
        nvs_set_u8(h, "bootcnt", 0);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "image validated; rollback guard cleared");
    }
    vTaskDelete(NULL);
}

void dfu_rollback_check_on_boot(void) {
    nvs_handle h;
    if (nvs_open("dfu", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    uint8_t pending = 0;
    if (nvs_get_u8(h, "pending", &pending) != ESP_OK || pending == 0) {
        nvs_close(h);
        return;
    }
    uint8_t cnt = 0;
    nvs_get_u8(h, "bootcnt", &cnt);
    cnt++;

    if (cnt >= DFU_BOOT_FAIL_THRESHOLD) {
        uint8_t prev = 0;
        nvs_get_u8(h, "prev", &prev);
        const esp_partition_t *p =
            esp_partition_find_first(ESP_PARTITION_TYPE_APP, (esp_partition_subtype_t)prev, NULL);
        if (p != NULL) {
            esp_ota_set_boot_partition(p);
            ESP_LOGE(TAG, "new image failed to validate; reverting to %s", p->label);
        }
        nvs_set_u8(h, "pending", 0);
        nvs_set_u8(h, "bootcnt", 0);
        nvs_commit(h);
        nvs_close(h);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else {
        nvs_set_u8(h, "bootcnt", cnt);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGW(TAG, "unvalidated image, boot attempt %d/%d", cnt, DFU_BOOT_FAIL_THRESHOLD);
    }
}

void dfu_rollback_arm_validation(void) {
    xTaskCreate(rollback_validate_task, "dfu_valid", 2048, NULL, 3, NULL);
}

#endif  // DFU_ROLLBACK_GUARD
