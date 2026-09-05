// user_main.c — bare-SDK entry for the rc-switch firmware (Tuya T1-2S-NL /
// BK7238), built directly on beken_freertos_sdk. No OpenBeken OS.
//
// The SDK's app_pre_start() brings up the wifi core + threads, then the app
// thread calls the (weak) user_main(); this strong definition overrides it —
// the same hook OpenBeken used, but here it is our whole application:
//   - drive relay (P9) + status LED (P6) to defaults before/independent of radio
//   - listen in 802.11 monitor mode for the RC vendor-IE beacon protocol
//   - apply 0x06 RelaysUpdate / 0x07 RelaysLedUpdate via the pure controller
//   - (DFU2 0x05 handling added in a later step)
//
// We own the radio outright — no wifi manager to fight for monitor mode.

#include <stdint.h>
#include <stddef.h>

#include "rtos_pub.h"
#include "gpio_pub.h"     // gpio_config / gpio_output + GMODE_* enum
#include "uart_pub.h"     // bk_printf
#include "wlan_ui_pub.h"  // bk_wlan_* monitor API + wifi_link_info_t / monitor_cb_t

#include "config.h"
#include "protocol.h"
#include "controller.h"

static controller_t g_ctrl;
static uint8_t g_myId = RCSW_MY_ID;
static uint32_t g_ms;                 // coarse ms clock (bumped in the main loop)

// diagnostics
static volatile uint32_t g_rxFrames, g_rxOurs, g_rxRelay, g_rxLed;

static uint32_t now_ms(void) { return g_ms; }

static void set_relay(int on) { gpio_output(RCSW_PIN_RELAY, on ? 1 : 0); }
static void set_led(int on)   { gpio_output(RCSW_PIN_LED, on ? 1 : 0); }

// Monitor RX runs in WLAN driver context — keep it short.
static void monitor_rx(uint8_t *data, int len, wifi_link_info_t *info) {
    (void)info;
    g_rxFrames++;
    if (len <= 0) return;
    relay_parse_result_t r = protocol_parse_beacon((const uint8_t *)data, (size_t)len, g_myId);
    if (!r.valid) return;
    g_rxOurs++;
    uint32_t now = now_ms();
    if (r.has_relay) {
        g_rxRelay++;
        controller_on_relay(&g_ctrl, r.relay_state, now);
        set_relay(r.relay_state);
    }
    if (r.has_led) {
        g_rxLed++;
        controller_on_led(&g_ctrl, r.led_state, now);
        set_led(r.led_state);
    }
}

static void apply_outputs(void) {
    set_relay(controller_relay_out(&g_ctrl, now_ms()));
    set_led(controller_led_out(&g_ctrl, now_ms()));
}

void user_main(void) {
    // Outputs first: apply defaults immediately.
    gpio_config(RCSW_PIN_RELAY, GMODE_OUTPUT);
    gpio_config(RCSW_PIN_LED, GMODE_OUTPUT);
    gpio_config(RCSW_PIN_BUTTON, GMODE_INPUT_PULLUP);  // claimed, never read

    controller_config_t cfg;
    cfg.default_relay = RCSW_DEFAULT_RELAY;
    cfg.default_led = RCSW_DEFAULT_LED;
    cfg.timeout_ms = RCSW_FALLBACK_MS;
    controller_init(&g_ctrl, cfg);
    apply_outputs();

    bk_printf("\r\nrc-switch (bare SDK) id=%d chan=%d\r\n", g_myId, RCSW_WIFI_CHANNEL);

    // The wifi core is up (app_pre_start ran before us). Enter monitor mode.
    rtos_delay_milliseconds(1500);
    bk_wlan_register_monitor_cb((monitor_cb_t)monitor_rx);
    bk_wlan_start_monitor();
    bk_wlan_set_channel_sync(RCSW_WIFI_CHANNEL);
    bk_printf("monitor started on chan %d\r\n", RCSW_WIFI_CHANNEL);

    // Main loop: advance the clock, run the fallback timers, log stats.
    uint32_t tick = 0;
    for (;;) {
        rtos_delay_milliseconds(200);
        g_ms += 200;
        apply_outputs();
        if (++tick % 5 == 0) {  // ~1 Hz
            bk_printf("rc-switch: frames=%u ours=%u relay=%u led=%u out(r=%d,l=%d)\r\n",
                      (unsigned)g_rxFrames, (unsigned)g_rxOurs,
                      (unsigned)g_rxRelay, (unsigned)g_rxLed,
                      controller_relay_out(&g_ctrl, now_ms()),
                      controller_led_out(&g_ctrl, now_ms()));
        }
    }
}
