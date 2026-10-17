// sdk_compat.h -- the small SDK differences that are not worth a whole seam.
//
// ESP8266_RTOS_SDK and ESP-IDF v6 diverge in a few places this firmware touches:
// promiscuous RX metadata field names, the network-interface init API, and the
// presence of a native OTA rollback. Each is a couple of lines, gathered here so
// the call sites stay readable.

#ifndef BULB_SDK_COMPAT_H
#define BULB_SDK_COMPAT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "sdkconfig.h"

#include "esp_ota_ops.h"

#if defined(CONFIG_IDF_TARGET_ESP8266)
#  include "tcpip_adapter.h"
#else
#  include "esp_netif.h"
#  include "esp_app_desc.h"
#endif

// ---- promiscuous RX metadata ------------------------------------------------

// 802.11 frame length from the RX control block. The ESP8266 splits this by
// modulation (beacons are legacy rate); ESP-IDF reports one sig_len. Both include
// the trailing FCS on some paths, which protocol_parse_beacon already tolerates.
static inline uint32_t bulb_rx_frame_len(const wifi_pkt_rx_ctrl_t *rx) {
#if defined(CONFIG_IDF_TARGET_ESP8266)
    return rx->sig_mode ? (uint32_t)rx->HT_length : (uint32_t)rx->legacy_length;
#else
    return (uint32_t)rx->sig_len;
#endif
}

// True when the hardware flagged this frame as damaged. The promiscuous path can
// deliver frames whose FCS check failed; our vendor IE carries no checksum of its
// own, so a bit error would otherwise be parsed as-is and flash the bulb to a
// wrong colour for one update.
static inline bool bulb_rx_bad(const wifi_pkt_rx_ctrl_t *rx) {
#if defined(CONFIG_IDF_TARGET_ESP8266)
    return rx->rxend_state != 0;
#else
    return rx->rx_state != 0;
#endif
}

// ---- network interface ------------------------------------------------------

static inline void bulb_netif_init(void) {
#if defined(CONFIG_IDF_TARGET_ESP8266)
    tcpip_adapter_init();
#else
    ESP_ERROR_CHECK(esp_netif_init());
#endif
}

// ESP-IDF needs an explicit default STA netif; the ESP8266 adapter creates one
// implicitly. Call after bulb_netif_init() and the event loop, before esp_wifi_init().
static inline void bulb_netif_create_sta(void) {
#if !defined(CONFIG_IDF_TARGET_ESP8266)
    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF") == NULL) {
        esp_netif_create_default_wifi_sta();
    }
#endif
}

// Looked up by ifkey rather than threaded through as state -- the handle is
// process-wide and this keeps the DFU path free of netif plumbing.
static inline void bulb_netif_set_hostname(const char *host) {
#if defined(CONFIG_IDF_TARGET_ESP8266)
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, host);
#else
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (n) esp_netif_set_hostname(n, host);
#endif
}

// ---- Wi-Fi interface id ----------------------------------------------------

// esp_wifi_set_config()'s interface argument: ESP_IF_WIFI_STA on the ESP8266 SDK,
// WIFI_IF_STA in ESP-IDF (the ESP_IF_* aliases are gone in v6).
#if defined(CONFIG_IDF_TARGET_ESP8266)
#  define BULB_WIFI_IF_STA ESP_IF_WIFI_STA
#else
#  define BULB_WIFI_IF_STA WIFI_IF_STA
#endif

// ---- app descriptor --------------------------------------------------------

// esp_ota_get_app_description() is gone in IDF v6 in favour of
// esp_app_get_description(); the ESP8266 SDK only has the former.
static inline const esp_app_desc_t *bulb_app_desc(void) {
#if defined(CONFIG_IDF_TARGET_ESP8266)
    return esp_ota_get_app_description();
#else
    return esp_app_get_description();
#endif
}

#endif  // BULB_SDK_COMPAT_H
