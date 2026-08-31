// sniffer.c — promiscuous RX. Keep this path lean: parse, and on a match hand
// off to the controller task. No flash, no blocking, no logging in the hot path.

#include "sniffer.h"

#include <string.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_log.h"

#include "config.h"
#include "bulb_config.h"
#include "protocol.h"
#include "controller.h"
#include "dfu2.h"

static const char *TAG = "sniffer";

// Highest BulbCommand seq processed since power-on (anti-replay). The RX callback
// is single-threaded (WiFi task), so plain statics need no locking.
static uint32_t s_highest_seq;
static bool s_seq_seen;

// Set once we have handed a Dfu2Request to the controller, so repeated broadcasts
// (which have no seq) do not queue the request again while DFU2 is spinning up.
static bool s_dfu2_notified;

// Handle a Dfu2Request addressed to us: if the advertised build id differs from
// our own running build id, copy the request out of the (aliased) RX buffer and
// hand it to the controller. If it matches, we are already on the target build.
static void handle_dfu2(const bulb_parse_result_t *r) {
    if (s_dfu2_notified) {
        return;
    }
    uint8_t mine[PROTO_DFU2_BUILD_ID_LEN];
    dfu2_own_build_id(mine);
    if (memcmp(mine, r->dfu2.build_id, r->dfu2.build_id_len) == 0) {
        return;  // already running the advertised build
    }

    dfu2_params_t p = {0};
    uint8_t sl = r->dfu2.ssid_len;
    if (sl > PROTO_DFU2_SSID_MAX) sl = PROTO_DFU2_SSID_MAX;
    memcpy(p.ssid, r->dfu2.ssid, sl);
    p.ssid[sl] = '\0';
    uint8_t pl = r->dfu2.pass_len;
    if (pl > PROTO_DFU2_PASS_MAX) pl = PROTO_DFU2_PASS_MAX;
    memcpy(p.pass, r->dfu2.pass, pl);
    p.pass[pl] = '\0';
    p.build_id_len = r->dfu2.build_id_len;
    memcpy(p.build_id, r->dfu2.build_id, r->dfu2.build_id_len);
    memcpy(p.server_ip, r->dfu2.server_ip, 4);
    p.server_port = r->dfu2.server_port;

    s_dfu2_notified = true;
    controller_notify_dfu2(&p);
}

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

    // The promiscuous path delivers frames even when the hardware FCS check
    // failed; rxend_state is nonzero for those. Our vendor IE carries no
    // checksum of its own, so a frame with a bit error would otherwise be
    // parsed as-is and can flash the bulb to a wrong color for one update.
    if (ppkt->rx_ctrl.rxend_state != 0) {
        return;
    }

    const uint32_t len = frame_length(&ppkt->rx_ctrl);

    // Read our id live from the config cache (a single atomic byte load) so a
    // SetConfig that rewrites CFG_BULB_ID takes effect on the very next frame
    // rather than only after a reboot.
    const uint8_t my_id = bulb_config_get_u8(CFG_BULB_ID);

    bulb_parse_result_t r = protocol_parse_beacon(ppkt->payload, len, my_id);
    if (!r.valid) {
        return;
    }

    // Dfu2Request (0x05): pull-based OTA. No seq gate — every broadcast is
    // re-evaluated; the build-id comparison + s_dfu2_notified provide idempotency.
    if (r.dfu2_requested) {
        handle_dfu2(&r);
        return;
    }

    if (r.is_command) {
        // Anti-replay: act only on the first sighting of a new-highest seq.
        // Retransmissions of the same command carry the same seq and are ignored.
        if (s_seq_seen && r.seq <= s_highest_seq) {
            return;
        }
        s_highest_seq = r.seq;
        s_seq_seen = true;

        // Note: 0x04 EnterDfuMode (r.dfu_requested) is intentionally NOT acted on
        // by this firmware — DFU is now pull-based via 0x05 Dfu2Request. The seq
        // is still consumed above so a later real command isn't reprocessed.
        if (r.reboot_requested) {
            controller_notify_reboot();
        } else if (r.has_config) {
            controller_notify_set_config(r.config_key, r.config_value, r.config_len);
        }
        return;  // command handled (or not addressed to us)
    }

    // RawLightUpdate (0x06): raw duties + per-channel periods, applied without the
    // curve/max-power cap. Distinct controller path from the curve-mapped colors.
    if (r.has_raw_entry) {
        controller_notify_raw_entry(r.r, r.g, r.b, r.ww, r.cw,
                                    r.period_r, r.period_g, r.period_b,
                                    r.period_ww, r.period_cw);
        return;
    }

    // Color-bearing packets (0x02, 0x03; legacy 0x01 if enabled). Any legacy
    // dfu_requested field is ignored (see above).
    if (r.has_entry) {
        controller_notify_entry(r.r, r.g, r.b, r.ww, r.cw);
    }
}

void sniffer_start(void) {
    uint8_t channel = bulb_config_get_u8(CFG_WIFI_CHANNEL);

    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "promiscuous on, channel %d, id %d", channel,
             bulb_config_get_u8(CFG_BULB_ID));
}

void sniffer_apply_channel(uint8_t channel) {
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    ESP_LOGI(TAG, "re-tuned to channel %d (err=%d)", channel, err);
}

void sniffer_stop(void) {
    esp_wifi_set_promiscuous(false);
}
