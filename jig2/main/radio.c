// radio.c — Wi-Fi bring-up shared by the DFU2 server and the air test, plus the
// raw beacon sender for RC vendor-IE packets.

#include "radio.h"

#include <string.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static bool s_inited;

void radio_init_once(void) {
    if (s_inited) return;
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    s_inited = true;
}

// Started, never-associated STA on a fixed channel: enough to inject frames.
void radio_start_sta(uint8_t channel) {
    radio_init_once();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(80);  // 0.25 dBm units: 20 dBm, the S3's maximum
}

// One beacon whose only IE is our vendor IE (OUI 52:43:68) wrapping `body`
// (packet tag + payload), as every RC device's parser requires.
int radio_send_beacon(const uint8_t *body, size_t len) {
    if (len > 70) return -1;  // ESP8266 bulbs drop larger frames; keep the shared limit
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    wifi_interface_t ifx = mode == WIFI_MODE_AP ? WIFI_IF_AP : WIFI_IF_STA;

    uint8_t mac[6];
    esp_wifi_get_mac(ifx, mac);
    uint8_t f[128];
    size_t o = 0;
    f[o++] = 0x80; f[o++] = 0x00;                    // beacon
    f[o++] = 0x00; f[o++] = 0x00;                    // duration
    memset(f + o, 0xFF, 6); o += 6;                  // DA broadcast
    memcpy(f + o, mac, 6); o += 6;                   // SA
    memcpy(f + o, mac, 6); o += 6;                   // BSSID
    f[o++] = 0x00; f[o++] = 0x00;                    // seq (driver fills)
    memset(f + o, 0, 8); o += 8;                     // timestamp
    f[o++] = 0x64; f[o++] = 0x00;                    // interval 100 TU
    f[o++] = 0x21; f[o++] = 0x00;                    // capability
    f[o++] = 0xDD; f[o++] = (uint8_t)(3 + len);      // vendor IE, the only IE
    f[o++] = 0x52; f[o++] = 0x43; f[o++] = 0x68;     // OUI
    memcpy(f + o, body, len); o += len;
    return esp_wifi_80211_tx(ifx, f, (int)o, true);
}
