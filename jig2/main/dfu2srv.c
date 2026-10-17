// dfu2srv.c — the deployment side of a DFU2 pull update, run from the jig:
//
//   * a soft AP for the target to join (what a real update network provides)
//   * a TCP DFU2 server (same wire protocol as rc-bulb-firmware's dfu2_server.py
//     and fw/main/dfu2.c): hello -> header -> image -> status byte
//   * 0x05 Dfu2Request beacons (what the base station broadcasts)
//
// The image (an rt_ota .rbl) is written to this S3's flash at DFU2SRV_IMG_ADDR
// beforehand with esptool and streamed from there, so nothing large crosses USB.
//
// Probe mode (img_len == 0): no image. Each target that connects is logged
// from its hello, ICMP-pinged while it waits for a header, then the connection
// is closed WITHOUT a header -- the target never writes flash, reports a bad
// header and reboots into its untouched firmware.

#include "dfu2srv.h"

#include <string.h>

#include "esp_event.h"
#include "esp_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "freertos/semphr.h"
#include "mbedtls/sha256.h"
#include "ping/ping_sock.h"
#include "radio.h"

#define DFU2_HELLO_MAGIC 0x32756C52u  // "Rlu2"
#define DFU2_HDR_MAGIC   0x686C4352u  // "RClh"
#define DFU2_HELLO_LEN   14            // magic(4) fmt(1) id(1) build_id(8)
#define DFU2_HDR_LEN     49            // magic(4) ver(1) len(4) sha256(32) build_id(8)
#define DFU2_PORT        3333
#define BEACON_PERIOD_MS 100

static dfu2srv_log_fn s_log;
static volatile bool s_running;
static uint32_t s_img_len;
static uint8_t  s_sha[32];
static uint8_t  s_build_id[8];
static uint8_t  s_dev_type;
static volatile uint8_t s_id_start, s_id_bounds;  // retargetable while running
static uint8_t  s_channel;  // AP + beacon channel: must be the one targets monitor
static char     s_ssid[33], s_pass[64];
static int      s_listen = -1;

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static bool hash_image(void) {
    static uint8_t buf[4096];
    mbedtls_sha256_context sh;
    mbedtls_sha256_init(&sh);
    mbedtls_sha256_starts(&sh, 0);
    for (uint32_t off = 0; off < s_img_len; off += sizeof buf) {
        uint32_t n = s_img_len - off < sizeof buf ? s_img_len - off : sizeof buf;
        if (esp_flash_read(NULL, buf, DFU2SRV_IMG_ADDR + off, n) != ESP_OK) return false;
        if (off == 0 && memcmp(buf, "RBL\0", 4) != 0) {
            s_log("DFU2 ERR no RBL image at 0x%x (write it with esptool first)", DFU2SRV_IMG_ADDR);
            return false;
        }
        mbedtls_sha256_update(&sh, buf, n);
    }
    mbedtls_sha256_finish(&sh, s_sha);
    mbedtls_sha256_free(&sh);
    return true;
}

static void wifi_ap_start(void) {
    radio_init_once();
    wifi_config_t ap = {0};
    strncpy((char *)ap.ap.ssid, s_ssid, sizeof ap.ap.ssid);
    ap.ap.ssid_len = (uint8_t)strlen(s_ssid);
    strncpy((char *)ap.ap.password, s_pass, sizeof ap.ap.password);
    ap.ap.channel = s_channel;
    ap.ap.max_connection = 4;
    ap.ap.authmode = s_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
}

// One beacon whose only IE is our vendor IE wrapping the 0x05 Dfu2Request.
static void send_request_beacon(void) {
    uint8_t body[70];
    size_t b = 0;
    size_t sl = strlen(s_ssid), pl = strlen(s_pass);
    body[b++] = 0x05;                        // Dfu2Request
    body[b++] = s_dev_type;
    body[b++] = s_id_start;
    body[b++] = s_id_bounds;
    body[b++] = 192; body[b++] = 168; body[b++] = 4; body[b++] = 1;  // soft-AP address
    body[b++] = DFU2_PORT & 0xFF; body[b++] = DFU2_PORT >> 8;
    body[b++] = 8; memcpy(body + b, s_build_id, 8); b += 8;
    body[b++] = (uint8_t)sl; memcpy(body + b, s_ssid, sl); b += sl;
    body[b++] = (uint8_t)pl; memcpy(body + b, s_pass, pl); b += pl;

    radio_send_beacon(body, b);
}

static void beacon_task(void *arg) {
    while (s_running) {
        send_request_beacon();
        vTaskDelay(pdMS_TO_TICKS(BEACON_PERIOD_MS));
    }
    vTaskDelete(NULL);
}

static bool recv_all(int s, uint8_t *buf, size_t len) {
    while (len > 0) {
        int n = recv(s, buf, len, 0);
        if (n <= 0) return false;
        buf += n;
        len -= (size_t)n;
    }
    return true;
}

// ---- ICMP ping of a connected target (probe mode) ---------------------------
static SemaphoreHandle_t s_ping_done;
static uint32_t s_ping_ok, s_ping_sum_ms, s_ping_max_ms;

static void on_ping_ok(esp_ping_handle_t h, void *a) {
    uint32_t ms = 0;
    esp_ping_get_profile(h, ESP_PING_PROF_TIMEGAP, &ms, sizeof ms);
    s_ping_ok++;
    s_ping_sum_ms += ms;
    if (ms > s_ping_max_ms) s_ping_max_ms = ms;
}
static void on_ping_timeout(esp_ping_handle_t h, void *a) {}
static void on_ping_end(esp_ping_handle_t h, void *a) { xSemaphoreGive(s_ping_done); }

static void ping_target(uint32_t ip_be, uint8_t id) {
    if (!s_ping_done) s_ping_done = xSemaphoreCreateBinary();
    s_ping_ok = s_ping_sum_ms = s_ping_max_ms = 0;
    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr.type = IPADDR_TYPE_V4;
    cfg.target_addr.u_addr.ip4.addr = ip_be;
    cfg.count = 3;
    cfg.interval_ms = 300;
    cfg.timeout_ms = 1000;
    esp_ping_callbacks_t cbs = {.on_ping_success = on_ping_ok,
                                .on_ping_timeout = on_ping_timeout,
                                .on_ping_end = on_ping_end};
    esp_ping_handle_t h;
    if (esp_ping_new_session(&cfg, &cbs, &h) != ESP_OK) {
        s_log("DFU2 PING id=%u: could not start ping", id);
        return;
    }
    esp_ping_start(h);
    xSemaphoreTake(s_ping_done, pdMS_TO_TICKS(6000));
    esp_ping_delete_session(h);
    s_log("DFU2 PING id=%u: %lu/3 replies, avg %lu ms, max %lu ms", id, (unsigned long)s_ping_ok,
          (unsigned long)(s_ping_ok ? s_ping_sum_ms / s_ping_ok : 0), (unsigned long)s_ping_max_ms);
}

static void serve_client(int c, uint32_t peer_ip_be) {
    struct timeval tv = {.tv_sec = 15};
    setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    uint8_t hello[DFU2_HELLO_LEN];
    if (!recv_all(c, hello, sizeof hello)) {
        s_log("DFU2 client: no hello");
        return;
    }
    uint32_t magic = hello[0] | (hello[1] << 8) | (hello[2] << 16) | ((uint32_t)hello[3] << 24);
    s_log("DFU2 hello: magic=%s fmt=%u id=%u build=%02x%02x%02x%02x%02x%02x%02x%02x",
          magic == DFU2_HELLO_MAGIC ? "ok" : "BAD", hello[4], hello[5], hello[6], hello[7],
          hello[8], hello[9], hello[10], hello[11], hello[12], hello[13]);
    if (magic != DFU2_HELLO_MAGIC) return;

    if (s_img_len == 0) {  // probe: answer the hello with pings, never a header
        ping_target(peer_ip_be, hello[5]);
        return;            // caller closes; target reboots into its own firmware
    }

    uint8_t hdr[DFU2_HDR_LEN];
    put_u32(hdr, DFU2_HDR_MAGIC);
    hdr[4] = 1;
    put_u32(hdr + 5, s_img_len);
    memcpy(hdr + 9, s_sha, 32);
    memcpy(hdr + 41, s_build_id, 8);
    if (send(c, hdr, sizeof hdr, 0) != sizeof hdr) return;

    static uint8_t buf[4096];
    uint32_t sent = 0;
    while (sent < s_img_len) {
        uint32_t n = s_img_len - sent < sizeof buf ? s_img_len - sent : sizeof buf;
        if (esp_flash_read(NULL, buf, DFU2SRV_IMG_ADDR + sent, n) != ESP_OK) break;
        size_t off = 0;
        while (off < n) {
            int w = send(c, buf + off, n - off, 0);
            if (w <= 0) goto out;
            off += (size_t)w;
        }
        sent += n;
        if (sent % (64 * 1024) == 0 || sent == s_img_len) {
            s_log("DFU2 sent %lu/%lu", (unsigned long)sent, (unsigned long)s_img_len);
        }
    }
out:
    if (sent != s_img_len) {
        s_log("DFU2 transfer aborted at %lu/%lu", (unsigned long)sent, (unsigned long)s_img_len);
        return;
    }
    // The target replies with one status byte (0 = OK) just before rebooting.
    tv.tv_sec = 120;
    setsockopt(c, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    uint8_t st = 0xFF;
    if (recv(c, &st, 1, 0) == 1) {
        s_log("DFU2 RESULT status=%u (%s)", st, st == 0 ? "OK, target rebooting" : "target reported failure");
    } else {
        s_log("DFU2 RESULT no status byte (target closed or timed out)");
    }
}

static void server_task(void *arg) {
    s_listen = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int yes = 1;
    setsockopt(s_listen, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(DFU2_PORT),
                            .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (bind(s_listen, (struct sockaddr *)&a, sizeof a) != 0 || listen(s_listen, 2) != 0) {
        s_log("DFU2 ERR cannot listen on :%d", DFU2_PORT);
        close(s_listen);
        s_listen = -1;
        vTaskDelete(NULL);
    }
    while (s_running) {
        struct sockaddr_in from;
        socklen_t fl = sizeof from;
        int c = accept(s_listen, (struct sockaddr *)&from, &fl);
        if (c < 0) continue;  // closed by dfu2srv_stop, or transient
        s_log("DFU2 client connected from %s", inet_ntoa(from.sin_addr));
        serve_client(c, from.sin_addr.s_addr);
        close(c);
    }
    vTaskDelete(NULL);
}

bool dfu2srv_start(const dfu2srv_params_t *p, dfu2srv_log_fn log) {
    if (s_running) {
        log("DFU2 ERR already running (DFU2 STOP first)");
        return false;
    }
    s_log = log;
    s_img_len = p->img_len;
    memcpy(s_build_id, p->build_id, 8);
    s_dev_type = p->dev_type;
    s_id_start = p->id_start;
    s_id_bounds = p->id_bounds;
    s_channel = p->channel ? p->channel : 11;
    strncpy(s_ssid, p->ssid, sizeof s_ssid - 1);
    strncpy(s_pass, p->pass, sizeof s_pass - 1);
    if (1 + 3 + 6 + 9 + 1 + strlen(s_ssid) + 1 + strlen(s_pass) > 70) {
        log("DFU2 ERR request body > 70 bytes; shorten ssid/pass");
        return false;
    }
    if (s_img_len == 0) {
        log("DFU2 probe mode: no image; targets are pinged then released untouched");
    } else {
        if (!hash_image()) return false;
        log("DFU2 image %lu bytes sha256 %02x%02x%02x%02x...", (unsigned long)s_img_len,
            s_sha[0], s_sha[1], s_sha[2], s_sha[3]);
    }

    wifi_ap_start();
    s_running = true;
    xTaskCreate(server_task, "dfu2srv", 6144, NULL, 5, NULL);
    xTaskCreate(beacon_task, "dfu2bcn", 3072, NULL, 5, NULL);
    log("DFU2 serving: ap '%s' ch %d, 192.168.4.1:%d, beacon type %u ids %u..%u",
        s_ssid, s_channel, DFU2_PORT, s_dev_type, s_id_start, s_id_start + s_id_bounds);
    return true;
}

bool dfu2srv_running(void) { return s_running; }

void dfu2srv_target(uint8_t id_start, uint8_t id_bounds) {
    s_id_start = id_start;
    s_id_bounds = id_bounds;
}

void dfu2srv_stop(void) {
    s_running = false;
    if (s_listen >= 0) {
        shutdown(s_listen, SHUT_RDWR);
        close(s_listen);
        s_listen = -1;
    }
}
