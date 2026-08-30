// dfu2.c — pull-based DFU client + OTA writer.
//
// Flow (all on a dedicated task with a large stack):
//   1. stop the sniffer, set a steady "in DFU" LED
//   2. join the AP named in the Dfu2Request (STA), acquire DHCP
//   3. TCP-connect OUT to the update server, retrying within a window
//   4. send a hello (magic | fmt | bulb_id | own_build_id)
//   5. read a header (magic | ver | image_len | sha256 | served_build_id) and
//      stream the image into the INACTIVE OTA slot, hashing as we go
//   6. verify transport SHA-256, let esp_ota_end validate the on-flash image,
//      and re-check the flashed image's build id == the served build id
//   7. flip the boot pointer, mark the rollback-guard pending, reply a status
//      byte, and reboot
//
// Reliability: the running slot is never written; any failure at any step reboots
// into the untouched current firmware, which will re-trigger on the next
// broadcast it hears. XIP safety is handled by the SDK (flash ops run from IRAM
// with cache + interrupts disabled per op); the IRAM-resident PWM ISR keeps the
// LEDs lit through the write.

#include "dfu2.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "pwm_output.h"
#include "sniffer.h"

static const char *TAG = "dfu2";

// ---- DFU2 wire framing -------------------------------------------------------
// Hello (bulb -> server): magic u32 LE | fmt u8 | bulb_id u8 | build_id[8] => 14
#define DFU2_HELLO_FMT 0x01
#define DFU2_HELLO_LEN (4 + 1 + 1 + PROTO_DFU2_BUILD_ID_LEN)   // 14
// Header (server -> bulb): magic u32 LE | version u8 | image_len u32 LE |
//                          sha256[32] | build_id[8]  => 49
#define DFU2_HDR_VERSION 0x01
#define DFU2_HDR_LEN (4 + 1 + 4 + 32 + PROTO_DFU2_BUILD_ID_LEN)  // 49

// status bytes sent back to the server (shared codes with dfu2_server.py)
enum {
    DFU_ST_OK = 0,
    DFU_ST_BAD_HEADER = 1,
    DFU_ST_OTA_BEGIN = 2,
    DFU_ST_RECV = 3,
    DFU_ST_SHA = 4,
    DFU_ST_OTA_END = 5,
    DFU_ST_SET_BOOT = 6,
    DFU_ST_BUILD_ID = 7,   // flashed image's build id != advertised target
    DFU_ST_NET = 8,        // could not join AP / reach server
};

// ---- wifi connect bookkeeping ----------------------------------------------
#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;
static char s_ip_str[16] = "0.0.0.0";

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();  // keep retrying until the outer timeout gives up
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        uint32_t a = evt->ip_info.ip.addr;  // network byte order; octet 1 is low byte
        snprintf(s_ip_str, sizeof(s_ip_str), "%u.%u.%u.%u",
                 (unsigned)(a & 0xff), (unsigned)((a >> 8) & 0xff),
                 (unsigned)((a >> 16) & 0xff), (unsigned)((a >> 24) & 0xff));
        ESP_LOGI(TAG, "DFU2: got IP %s", s_ip_str);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static bool connect_to_ap(uint8_t my_id, const dfu2_params_t *p) {
    s_wifi_events = xEventGroupCreate();

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL);

    // Hostname must be set before the DHCP client starts (i.e. before connect).
    char host[32];
    snprintf(host, sizeof(host), "%s%u", DFU_HOSTNAME_PREFIX, (unsigned)my_id);
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, host);
    ESP_LOGI(TAG, "DFU2 joining AP '%s' as %s", p->ssid, host);

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, p->ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, p->pass, sizeof(wc.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wc));
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(DFU2_CONNECT_TIMEOUT_MS));
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

// This image's build id = SHA-256(app version string)[:PROTO_DFU2_BUILD_ID_LEN].
// The version string (esp_app_desc.version) is populated by the build from
// version.txt (a random per-build token; see build.sh); app_elf_sha256 is NOT
// stamped by this SDK toolchain, so we hash the version instead. Cached after the
// first call.
void dfu2_own_build_id(uint8_t out[PROTO_DFU2_BUILD_ID_LEN]) {
    static uint8_t cached[PROTO_DFU2_BUILD_ID_LEN];
    static bool have;
    if (!have) {
        const esp_app_desc_t *d = esp_ota_get_app_description();
        uint8_t digest[32];
        mbedtls_sha256_ret((const unsigned char *)d->version, strlen(d->version), digest, 0);
        memcpy(cached, digest, PROTO_DFU2_BUILD_ID_LEN);
        have = true;
    }
    memcpy(out, cached, PROTO_DFU2_BUILD_ID_LEN);
}

// Open a TCP connection to the server, retrying "connection refused" within the
// window (the server may not be listening the instant we join). Returns a
// connected socket, or -1.
static int connect_to_server(const dfu2_params_t *p) {
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(p->server_port);
    // server_ip octets are in dotted order (a.b.c.d).
    uint32_t addr = ((uint32_t)p->server_ip[0]) | ((uint32_t)p->server_ip[1] << 8) |
                    ((uint32_t)p->server_ip[2] << 16) | ((uint32_t)p->server_ip[3] << 24);
    dst.sin_addr.s_addr = addr;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(DFU2_TCP_CONNECT_WINDOW_MS);
    for (;;) {
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock >= 0) {
            if (connect(sock, (struct sockaddr *)&dst, sizeof(dst)) == 0) {
                return sock;
            }
            close(sock);
        }
        if (xTaskGetTickCount() >= deadline) {
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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

// Connect to the server, do the hello/header handshake, receive+verify the image,
// and (on success) set the boot partition. Returns a DFU_ST_* code; on OK the
// caller reboots into the new image.
static int run_pull(uint8_t my_id, const dfu2_params_t *p) {
    int sock = connect_to_server(p);
    if (sock < 0) {
        ESP_LOGE(TAG, "could not reach server %u.%u.%u.%u:%u",
                 p->server_ip[0], p->server_ip[1], p->server_ip[2], p->server_ip[3],
                 (unsigned)p->server_port);
        return DFU_ST_NET;
    }

    // Guard against a stalled server mid-transfer.
    struct timeval rto = {.tv_sec = DFU2_RECV_TIMEOUT_MS / 1000, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof(rto));

    // ---- hello: magic | fmt | bulb_id | own_build_id[8] ----
    uint8_t hello[DFU2_HELLO_LEN];
    hello[0] = (uint8_t)(DFU2_HELLO_MAGIC);
    hello[1] = (uint8_t)(DFU2_HELLO_MAGIC >> 8);
    hello[2] = (uint8_t)(DFU2_HELLO_MAGIC >> 16);
    hello[3] = (uint8_t)(DFU2_HELLO_MAGIC >> 24);
    hello[4] = DFU2_HELLO_FMT;
    hello[5] = my_id;
    dfu2_own_build_id(&hello[6]);
    if (send(sock, hello, sizeof(hello), 0) != (int)sizeof(hello)) {
        close(sock);
        return DFU_ST_NET;
    }

    int status = DFU_ST_BAD_HEADER;
    uint8_t hdr[DFU2_HDR_LEN];
    if (recv_exact(sock, hdr, sizeof(hdr))) {
        uint32_t magic = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) |
                         ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
        uint8_t version = hdr[4];
        uint32_t image_len = (uint32_t)hdr[5] | ((uint32_t)hdr[6] << 8) |
                             ((uint32_t)hdr[7] << 16) | ((uint32_t)hdr[8] << 24);
        const uint8_t *sha = &hdr[9];
        const uint8_t *served_build_id = &hdr[9 + 32];

        const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
        if (magic != DFU_OTA_MAGIC || version != DFU2_HDR_VERSION ||
            image_len == 0 || part == NULL || image_len > part->size) {
            ESP_LOGE(TAG, "bad header (magic=%08x ver=%d len=%u)",
                     (unsigned)magic, version, (unsigned)image_len);
            status = DFU_ST_BAD_HEADER;
        } else if (memcmp(served_build_id, p->build_id, p->build_id_len) != 0) {
            // The server offered a build other than the one we were told to fetch.
            ESP_LOGE(TAG, "served build id != advertised target; refusing");
            status = DFU_ST_BUILD_ID;
        } else {
            ESP_LOGI(TAG, "receiving %u bytes into %s", (unsigned)image_len, part->label);
            status = receive_image(sock, part, image_len, sha);
            if (status == DFU_ST_OK) {
                // We already verified served_build_id == the advertised target
                // (above) and the transport SHA-256 (in receive_image), so the
                // flashed image is exactly the one the server computed its build
                // id from — no separate partition re-read is needed (and the
                // ESP8266 SDK does not stamp app_elf_sha256 anyway).
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

// Bundle my_id with the request so a single malloc'd block is handed to the task
// (freed there).
typedef struct {
    uint8_t my_id;
    dfu2_params_t params;
} dfu2_job_t;

static void dfu2_job_task(void *arg) {
    dfu2_job_t *job = (dfu2_job_t *)arg;
    ESP_LOGW(TAG, "entering DFU2 mode (bulb %u)", (unsigned)job->my_id);

    sniffer_stop();
    // Steady, known LED state during the update: 50% blue signals "in DFU mode".
    pwm_output_set(0.0f, 0.0f, 0.5f, 0.0f, 0.0f);

    if (!connect_to_ap(job->my_id, &job->params)) {
        ESP_LOGE(TAG, "failed to join AP; rebooting");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    int status = run_pull(job->my_id, &job->params);
    ESP_LOGI(TAG, "DFU2 finished with status %d; rebooting", status);
    vTaskDelay(pdMS_TO_TICKS(500));  // let logs/reply flush
    esp_restart();
}

void dfu2_start(uint8_t my_id, const dfu2_params_t *params) {
    // Dedicated task: the DFU path needs a much larger stack than the light
    // controller (wifi connect + lwIP sockets + esp_ota_write + mbedTLS SHA).
    dfu2_job_t *job = malloc(sizeof(*job));
    if (job == NULL) {
        ESP_LOGE(TAG, "out of memory starting DFU2; rebooting");
        esp_restart();
    }
    job->my_id = my_id;
    memcpy(&job->params, params, sizeof(*params));
    if (xTaskCreate(dfu2_job_task, "dfu2", 8192, job, 6, NULL) != pdPASS) {
        free(job);
        ESP_LOGE(TAG, "failed to start DFU2 task");
    }
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

void dfu2_rollback_check_on_boot(void) {
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
        const esp_partition_t *pt =
            esp_partition_find_first(ESP_PARTITION_TYPE_APP, (esp_partition_subtype_t)prev, NULL);
        if (pt != NULL) {
            esp_ota_set_boot_partition(pt);
            ESP_LOGE(TAG, "new image failed to validate; reverting to %s", pt->label);
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

void dfu2_rollback_arm_validation(void) {
    xTaskCreate(rollback_validate_task, "dfu2_valid", 2048, NULL, 3, NULL);
}

#endif  // DFU_ROLLBACK_GUARD
