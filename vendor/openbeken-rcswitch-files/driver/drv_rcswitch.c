// drv_rcswitch.c — RC broadcast-protocol relay switch for the Tuya T1-2S-NL
// (BK7238), running on the OpenBeken base.
//
// Listens in 802.11 monitor mode for the RC vendor-IE beacon protocol and acts
// on 0x06 RelaysUpdate / 0x07 RelaysLedUpdate (see protocol.*), applying the
// dual-default / dual-fallback-timer logic in controller.*. The physical
// button has no function (per spec).
//
// Monitor mode is started a few seconds after boot, once OpenBeken has brought
// the wifi core up — bk_wlan_start_monitor() tears down STA/AP and enters
// monitor, mirroring the SDK's CLI 'monitor' command (which works from a warm
// core; the crash-loop seen on a cold core does not apply here).

#include "../new_common.h"
#include "../logging/logging.h"
#include "../hal/hal_pins.h"
#include "drv_rcswitch.h"
#include "protocol.h"
#include "controller.h"

// The Beken monitor API (bk_wlan_start_monitor / bk_wlan_register_monitor_cb /
// bk_wlan_set_channel_sync) and wifi_link_info_t / monitor_cb_t come in via
// new_common.h -> wlan_ui_pub.h. Logging goes through OpenBeken's ADDLOG_INFO.

// ---- hardware (T1-2S-NL) : OpenBeken pin index == BK GPIO number ----
#define RCSW_PIN_RELAY  9   // P9, active-high
#define RCSW_PIN_LED    6   // P6, status LED
#define RCSW_PIN_BUTTON 8   // P8, unused by design

// ---- spike configuration (will move to OpenBeken cfg later) ----
#define RCSW_MY_ID       5
#define RCSW_CHANNEL     11
#define RCSW_TIMEOUT_MS  15000
#define RCSW_DEFAULT_RELAY 0
#define RCSW_DEFAULT_LED   0

static controller_t g_ctrl;
static uint8_t g_myId = RCSW_MY_ID;
static int g_monitorStarted = 0;
static int g_secs = 0;

// RX stats (diagnostics)
static volatile uint32_t g_rxFrames = 0;
static volatile uint32_t g_rxOurs = 0;
static volatile uint32_t g_rxRelay = 0;
static volatile uint32_t g_rxLed = 0;

// Millisecond clock for the controller. 1s granularity (from the OnEverySecond
// counter) is plenty for the 15s fallback window and avoids depending on
// FreeRTOS tick headers from this driver.
static uint32_t rcsw_now_ms(void) {
    return (uint32_t)g_secs * 1000u;
}

// Monitor RX runs in WLAN driver context: keep it short.
static void rcsw_monitor_rx(uint8_t *data, int len, wifi_link_info_t *info) {
    (void)info;
    g_rxFrames++;
    if (len <= 0) return;
    relay_parse_result_t r = protocol_parse_beacon((const uint8_t *)data, (size_t)len, g_myId);
    if (!r.valid) return;
    g_rxOurs++;
    uint32_t now = rcsw_now_ms();
    if (r.has_relay) {
        g_rxRelay++;
        controller_on_relay(&g_ctrl, r.relay_state, now);
        HAL_PIN_SetOutputValue(RCSW_PIN_RELAY, r.relay_state ? 1 : 0);
    }
    if (r.has_led) {
        g_rxLed++;
        controller_on_led(&g_ctrl, r.led_state, now);
        HAL_PIN_SetOutputValue(RCSW_PIN_LED, r.led_state ? 1 : 0);
    }
}

static void rcsw_apply_outputs(void) {
    uint32_t now = rcsw_now_ms();
    HAL_PIN_SetOutputValue(RCSW_PIN_RELAY, controller_relay_out(&g_ctrl, now) ? 1 : 0);
    HAL_PIN_SetOutputValue(RCSW_PIN_LED, controller_led_out(&g_ctrl, now) ? 1 : 0);
}

void RCSwitch_Init(void) {
    controller_config_t cfg;
    cfg.default_relay = RCSW_DEFAULT_RELAY;
    cfg.default_led = RCSW_DEFAULT_LED;
    cfg.timeout_ms = RCSW_TIMEOUT_MS;
    controller_init(&g_ctrl, cfg);

    HAL_PIN_Setup_Output(RCSW_PIN_RELAY);
    HAL_PIN_Setup_Output(RCSW_PIN_LED);
    HAL_PIN_Setup_Input_Pullup(RCSW_PIN_BUTTON);  // claimed, never read
    rcsw_apply_outputs();  // defaults, before radio

    ADDLOG_INFO(LOG_FEATURE_GENERAL, "RCSwitch: init id=%d chan=%d (monitor starts in a few s)\r\n",
              g_myId, RCSW_CHANNEL);
}

// Called ~1 Hz. Start monitor once the wifi core is warm, then keep the channel
// pinned and run the fallback timers.
void RCSwitch_OnEverySecond(void) {
    g_secs++;

    // Re-assert monitor mode + channel each second. OpenBeken actively manages
    // wifi (it brings up a config AP), which can pull the radio out of monitor;
    // re-asserting keeps us sniffing on our channel.
    if (g_secs >= 3) {
        bk_wlan_register_monitor_cb(rcsw_monitor_rx);
        bk_wlan_start_monitor();
        bk_wlan_set_channel_sync(RCSW_CHANNEL);
        g_monitorStarted = 1;
    }

    // Apply outputs from the controller so the per-packet-type fallback timers
    // take effect: if no RelaysUpdate / RelaysLedUpdate is heard for the window,
    // the relay / LED revert to their configured defaults.
    rcsw_apply_outputs();

    ADDLOG_INFO(LOG_FEATURE_GENERAL, "RCSwitch: frames=%u ours=%u relay=%u led=%u\r\n",
              (unsigned)g_rxFrames, (unsigned)g_rxOurs, (unsigned)g_rxRelay, (unsigned)g_rxLed);
}

void RCSwitch_RunQuickTick(void) {
    // Outputs are driven directly from RX and once per second; nothing needed
    // at high frequency for the spike.
}

void RCSwitch_Stop(void) {
    if (g_monitorStarted) {
        bk_wlan_stop_monitor();
        g_monitorStarted = 0;
    }
}

void RCSwitch_AppendInformationToHTTPIndexPage(void *request, int bPreState) {
    (void)request; (void)bPreState;
}
