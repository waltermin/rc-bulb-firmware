// sniffer.c — promiscuous RX. Keep this path lean: parse, and on a match hand
// off to the controller task. No flash, no blocking, no logging in the hot path.

#include "sniffer.h"

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_log.h"

#include "config.h"
#include "protocol.h"
#include "controller.h"

static const char *TAG = "sniffer";

static uint8_t s_my_id;

// Extract the 802.11 frame length from the RX control metadata. Beacons are
// sent at legacy rates, so legacy_length is the payload length; fall back to
// HT_length for HT frames.
static inline uint32_t frame_length(const wifi_pkt_rx_ctrl_t *rx) {
    return rx->sig_mode ? (uint32_t)rx->HT_length : (uint32_t)rx->legacy_length;
}

static void sniffer_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT || buf == NULL) {
        return;
    }
    const wifi_promiscuous_pkt_t *ppkt = (const wifi_promiscuous_pkt_t *)buf;
    const uint32_t len = frame_length(&ppkt->rx_ctrl);

    bulb_parse_result_t r = protocol_parse_beacon(ppkt->payload, len, s_my_id);
    if (!r.valid) {
        return;
    }
    if (r.dfu_requested) {
        controller_notify_dfu();
        return;  // ignore any color in the same frame; DFU takes over
    }
    if (r.has_entry) {
        controller_notify_entry(r.r, r.g, r.b, r.ww, r.cw);
    }
}

void sniffer_start(uint8_t my_id) {
    s_my_id = my_id;

    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "promiscuous on, channel %d, id %d", WIFI_CHANNEL, my_id);
}

void sniffer_stop(void) {
    esp_wifi_set_promiscuous(false);
}
