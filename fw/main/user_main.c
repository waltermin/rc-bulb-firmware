// user_main.c — bare-SDK entry for the rc-switch firmware (Tuya T1-2S-NL /
// BK7238), built directly on beken_freertos_sdk. No OpenBeken OS.
//
// The SDK's app_pre_start() brings up the wifi core + threads, then the app
// thread calls the (weak) user_main(); this strong definition overrides it —
// the same hook OpenBeken used, but here it is our whole application:
//   - drive relay (P9) + status LED (P6) to defaults before/independent of radio
//   - listen in 802.11 monitor mode for the RC vendor-IE beacon protocol
//   - apply 0x40 RelaysUpdate / 0x41 RelaysLedUpdate via the pure controller
//   - (DFU2 0x05 handling added in a later step)
//
// We own the radio outright — no wifi manager to fight for monitor mode.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "rtos_pub.h"
#include "gpio_pub.h"     // gpio_config / gpio_output + GMODE_* enum
#include "uart_pub.h"     // bk_printf
#include "wlan_ui_pub.h"  // bk_wlan_* monitor API + wifi_link_info_t / monitor_cb_t

#include "config.h"
#include "protocol.h"
#include "controller.h"
#include "dfu2.h"
#include "settings.h"

static controller_t g_ctrl;
static rcsw_settings_t g_settings;    // persisted config (loaded at boot)
static uint8_t g_myId = RCSW_MY_ID;
static uint8_t g_curChannel;          // channel the monitor is currently on
static uint32_t g_ms;                 // coarse ms clock (bumped in the main loop)

// diagnostics (bench visibility on UART1; cheap enough to keep in production)
static volatile uint32_t g_rxFrames, g_rxOurs, g_rxRelay, g_rxLed, g_rxDfu;

// DFU2 request captured in the RX callback, acted on by the main loop.
static volatile int g_dfuPending;
static dfu2_params_t g_dfuParams;

// 0x04 BulbCommand state. seq is the anti-replay watermark (highest command seq
// seen). SetConfig / Reboot are captured in the RX callback and acted on by the
// main loop (flash writes + reboot must not run in WLAN driver context).
static volatile uint32_t g_lastSeq;
static volatile int g_cfgPending, g_rebootPending;
static volatile uint16_t g_cfgKey;
static volatile uint8_t g_cfgLen;
static uint8_t g_cfgVal[PROTO_CONFIG_VALUE_MAX];

extern void bk_reboot(void);

static uint32_t now_ms(void) { return g_ms; }

// Push the current g_settings into the live state: id used by the parser, the
// controller's defaults/fallback window, and (if changed) the monitor channel.
static void apply_settings_live(void) {
    g_myId = g_settings.my_id;
    g_ctrl.cfg.default_relay = g_settings.default_relay ? true : false;
    g_ctrl.cfg.default_led   = g_settings.default_led ? true : false;
    g_ctrl.cfg.timeout_ms    = g_settings.fallback_ms;
    if (g_curChannel != g_settings.wifi_channel) {
        g_curChannel = g_settings.wifi_channel;
        bk_wlan_set_channel_sync(g_curChannel);
    }
}

static void set_relay(int on) { gpio_output(RCSW_PIN_RELAY, on ? 1 : 0); }
static void set_led(int on)   { gpio_output(RCSW_PIN_LED, on ? 1 : 0); }

// The RX-callback -> main-loop handshakes pass multi-word payloads guarded by a
// single volatile flag. volatile orders only volatile accesses, so a compiler
// barrier is needed to keep the payload stores/loads on the correct side of the
// flag on both ends (single core, so a compiler fence is sufficient).
#define rcsw_barrier() __asm volatile("" ::: "memory")

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
    if (r.dfu2_requested && !g_dfuPending && !dfu2_in_progress()) {
        // Self-terminating: only act if the advertised build differs from ours.
        uint8_t mine[PROTO_DFU2_BUILD_ID_LEN];
        dfu2_own_build_id(mine);
        uint8_t n = r.dfu2.build_id_len;
        if (n > PROTO_DFU2_BUILD_ID_LEN) n = PROTO_DFU2_BUILD_ID_LEN;
        if (memcmp(r.dfu2.build_id, mine, n) != 0) {
            g_rxDfu++;
            // Copy the aliased packet fields before the RX buffer is reused.
            memset(&g_dfuParams, 0, sizeof(g_dfuParams));
            memcpy(g_dfuParams.build_id, r.dfu2.build_id, n);
            g_dfuParams.build_id_len = n;
            if (r.dfu2.ssid_len < sizeof(g_dfuParams.ssid))
                memcpy(g_dfuParams.ssid, r.dfu2.ssid, r.dfu2.ssid_len);
            if (r.dfu2.pass_len < sizeof(g_dfuParams.pass))
                memcpy(g_dfuParams.pass, r.dfu2.pass, r.dfu2.pass_len);
            memcpy(g_dfuParams.server_ip, r.dfu2.server_ip, 4);
            g_dfuParams.server_port = r.dfu2.server_port;
            rcsw_barrier();   // payload fully written before the flag publishes it
            g_dfuPending = 1;
        }
    }

    // 0x04 BulbCommand: SetConfig / Reboot, gated on the anti-replay watermark.
    // is_command is set for any well-formed command; reboot_requested/has_config
    // only when addressed to us. Flash writes + reboot happen in the main loop.
    if (r.is_command && r.seq > g_lastSeq) {
        if (r.has_config) {
            if (!g_cfgPending) {
                uint8_t n = r.config_len;
                if (n > sizeof(g_cfgVal)) n = sizeof(g_cfgVal);
                g_cfgKey = r.config_key;
                g_cfgLen = n;
                memcpy(g_cfgVal, r.config_value, n);
                rcsw_barrier();   // payload fully written before the flag publishes it
                g_cfgPending = 1;
                g_lastSeq = r.seq;   // advance only once we've captured it
            }
            // else: busy — don't advance; a rebroadcast (same seq) retries.
        } else {
            if (r.reboot_requested) g_rebootPending = 1;
            g_lastSeq = r.seq;       // track no-effect / not-for-us commands too
        }
    }
}

static void apply_outputs(void) {
    set_relay(controller_relay_out(&g_ctrl, now_ms()));
    set_led(controller_led_out(&g_ctrl, now_ms()));
}

// SDK debug print port (defaults to UART2); redirect to UART1 so the bench jig
// (which reads UART1) sees our logs.
extern int uart_print_port;

void user_main(void) {
    uart_print_port = UART1_PORT;

    // Outputs first: apply defaults immediately.
    gpio_config(RCSW_PIN_RELAY, GMODE_OUTPUT);
    gpio_config(RCSW_PIN_LED, GMODE_OUTPUT);
    gpio_config(RCSW_PIN_BUTTON, GMODE_INPUT_PULLUP);  // claimed, never read

    // Load persisted config (falls back to config.h defaults if unset/invalid).
    rcsw_settings_load(&g_settings);
    g_myId = g_settings.my_id;
    g_curChannel = g_settings.wifi_channel;

    controller_config_t cfg;
    cfg.default_relay = g_settings.default_relay ? true : false;
    cfg.default_led = g_settings.default_led ? true : false;
    cfg.timeout_ms = g_settings.fallback_ms;
    controller_init(&g_ctrl, cfg);
    apply_outputs();

    bk_printf("\r\nrc-switch (bare SDK) ver=%s id=%d chan=%d\r\n", RCSW_VERSION, g_myId, g_curChannel);

    // The wifi core is up (app_pre_start ran before us). Enter monitor mode.
    rtos_delay_milliseconds(1500);
    bk_wlan_register_monitor_cb((monitor_cb_t)monitor_rx);
    bk_wlan_start_monitor();
    bk_wlan_set_channel_sync(g_curChannel);
    bk_printf("monitor started on chan %d\r\n", g_curChannel);

    // Main loop: advance the clock, run the fallback timers, drive DFU2.
    uint32_t tick = 0;
    for (;;) {
        rtos_delay_milliseconds(200);
        g_ms += 200;

        if (dfu2_in_progress()) {
            // Fast blink P6 as the "updating" indicator (single LED, so a
            // pattern rather than the bulb's steady blue). Don't touch the
            // relay: it holds its last state through the update.
            set_led((tick & 1) ? 1 : 0);
        } else if (g_dfuPending) {
            // Consume while still flagged pending: the RX callback won't touch
            // g_dfuParams until the flag drops, and dfu2_start() marks
            // in-progress before returning, so clearing the flag last leaves no
            // window where a new 0x05 can overwrite the params mid-use.
            bk_printf("rc-switch: DFU2 requested -> ap='%s' %u.%u.%u.%u:%u\r\n",
                      g_dfuParams.ssid, g_dfuParams.server_ip[0], g_dfuParams.server_ip[1],
                      g_dfuParams.server_ip[2], g_dfuParams.server_ip[3], g_dfuParams.server_port);
            dfu2_start(g_myId, &g_dfuParams);
            rcsw_barrier();
            g_dfuPending = 0;
        } else {
            apply_outputs();  // fallback timers when no packets arrive
        }

        // 0x04 SetConfig: apply + persist (only if a value actually changed),
        // then push the new values into the live state. Never while a DFU is
        // streaming: rcsw_settings_save toggles the global flash protection the
        // OTA writer depends on. (A DFU always ends in a reboot; the pending
        // command is dropped and the base station's rebroadcast retries.)
        if (g_cfgPending && !dfu2_in_progress()) {
            // Copy everything out BEFORE releasing the slot — clearing the flag
            // first would let the RX callback overwrite key/len/value mid-read.
            uint16_t key = g_cfgKey;
            uint8_t n = g_cfgLen;
            uint8_t val[PROTO_CONFIG_VALUE_MAX];
            if (n > sizeof(val)) n = sizeof(val);
            memcpy(val, g_cfgVal, n);
            rcsw_barrier();
            g_cfgPending = 0;
            if (rcsw_settings_apply(&g_settings, key, val, n)) {
                bk_printf("rc-switch: SetConfig key=0x%04x len=%d -> saving\r\n",
                          (unsigned)key, n);
                rcsw_settings_save(&g_settings);
                apply_settings_live();
            }
        }

        // 0x04 Reboot: honored (rt_ota check runs on the next boot). Not while
        // a DFU is streaming — that path reboots on its own when it finishes.
        if (g_rebootPending && !dfu2_in_progress()) {
            g_rebootPending = 0;
            bk_printf("rc-switch: reboot command -> rebooting\r\n");
            rtos_delay_milliseconds(50);
            bk_reboot();
        }

        if (++tick % 5 == 0) {  // ~1 Hz
            bk_printf("rc-switch: frames=%u ours=%u relay=%u led=%u dfu=%u out(r=%d,l=%d)\r\n",
                      (unsigned)g_rxFrames, (unsigned)g_rxOurs,
                      (unsigned)g_rxRelay, (unsigned)g_rxLed, (unsigned)g_rxDfu,
                      controller_relay_out(&g_ctrl, now_ms()),
                      controller_led_out(&g_ctrl, now_ms()));
        }
    }
}
