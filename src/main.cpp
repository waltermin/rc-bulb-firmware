// main.cpp — P0/P1 spike: monitor-mode RX -> protocol parser -> relay/LED.
//
// Startup ordering per design: outputs go to DEFAULT before the radio is
// touched. UART1 (TX1/RX1, the bench jig's port) carries logs + a minimal
// console. The button (P8) is deliberately not read — spec: no function.

#include <Arduino.h>

#define SPIKE_USE_WIFI 1
#if SPIKE_USE_WIFI
#include <WiFi.h>  // used ONLY to initialize the WLAN stack before monitor mode
#endif

extern "C" {
#include "config.h"
#include "controller.h"
#include "protocol.h"

// Beken SDK monitor-mode API (vendored by LibreTiny; declared here to avoid
// depending on SDK header include paths from C++).
typedef void (*monitor_cb_t)(uint8_t *data, int len, void *unused);
void bk_wlan_register_monitor_cb(monitor_cb_t fn);
int bk_wlan_start_monitor(void);
int bk_wlan_stop_monitor(void);
int bk_wlan_set_channel(int channel);

// Bring up the BDK wifi core WITHOUT enabling STA/AP. These are what the WiFi
// library calls internally on first mode change; calling them directly lets us
// enter monitor mode from a clean core (WiFi.mode(WIFI_STA) enables STA, which
// then fights bk_wlan_start_monitor and crash-loops via MM_RESET_REQ).
void func_init_extended(void);
void app_pre_start(void);
void *xTaskGetHandle(const char *name);  // FreeRTOS: check init_thread completion
}

static controller_t ctrl;
static uint8_t my_id = RELAY_DEFAULT_ID;

// RX stats for the console
static volatile uint32_t rx_frames = 0;      // all frames seen in monitor mode
static volatile uint32_t rx_ours = 0;        // valid protocol frames
static volatile uint32_t rx_relay_pkts = 0;  // 0x06 seen
static volatile uint32_t rx_led_pkts = 0;    // 0x07 seen
static volatile bool dump_next = false;      // console 'dump': hexdump next valid frame

// Monitor callback runs in WLAN driver context: parse, poke the controller,
// defer everything slow to loop().
static void monitor_rx(uint8_t *data, int len, void *) {
    rx_frames++;
    if (len <= 0) return;

    relay_parse_result_t r = protocol_parse_beacon(data, (size_t)len, my_id);
    if (!r.valid) return;
    rx_ours++;

    const uint32_t now = millis();
    if (r.has_relay) {
        rx_relay_pkts++;
        controller_on_relay(&ctrl, r.relay_state, now);
        // DIAGNOSTIC: drive the relay pin straight from RX so a jig broadcast
        // toggling it (visible via jig GPIO16 sense) proves monitor RX works,
        // independent of UART.
        digitalWrite(GPIO_RELAY, r.relay_state ? HIGH : LOW);
    }
    if (r.has_led) {
        rx_led_pkts++;
        controller_on_led(&ctrl, r.led_state, now);
    }

    if (dump_next) {
        dump_next = false;
        Serial.printf("[rx] %d bytes:", len);
        for (int i = 0; i < len && i < 96; i++) Serial.printf(" %02X", data[i]);
        Serial.println();
    }
}

static void apply_outputs() {
    const uint32_t now = millis();
    const bool relay = controller_relay_out(&ctrl, now);
    const bool led = controller_led_out(&ctrl, now);
    digitalWrite(GPIO_RELAY, relay ? HIGH : LOW);
    digitalWrite(GPIO_LED, (led ^ (bool)LED_ACTIVE_LOW) ? HIGH : LOW);
}

void setup() {
    // Outputs first, radio later: defaults apply within ms of power-on.
    controller_config_t cfg = {
        .default_relay = (bool)DEFAULT_RELAY,
        .default_led = (bool)DEFAULT_LED,
        .timeout_ms = FALLBACK_TIMEOUT_MS,
    };
    controller_init(&ctrl, cfg);
    pinMode(GPIO_RELAY, OUTPUT);
    pinMode(GPIO_LED, OUTPUT);
    pinMode(GPIO_BUTTON, INPUT_PULLUP);  // claimed but never read (spec: no function)
    apply_outputs();

    Serial1.begin(115200);  // UART1 = TX1/RX1 = the jig's port
    Serial1.println();
    Serial1.printf("rc-switch-firmware P0 spike | id=%u chan=%u\r\n", my_id, WIFI_CHANNEL);

    // The BDK's WLAN stack (func_init_extended/app_pre_start) is initialized
    // lazily by the WiFi library; bk_wlan_start_monitor() on a cold radio
    // hard-faults. Bring the stack up as STA (associates with nothing), then
    // switch to monitor.
#if SPIKE_USE_WIFI
    // Init the wifi core (no STA/AP), then enter monitor mode.
    func_init_extended();
    app_pre_start();
    for (int i = 0; i < 500 && xTaskGetHandle("init_thread"); i++) delay(10);
    delay(200);
    Serial1.println("wlan core up");

    bk_wlan_register_monitor_cb(monitor_rx);
    bk_wlan_start_monitor();
    bk_wlan_set_channel(WIFI_CHANNEL);
    Serial1.println("monitor mode started");
#else
    Serial1.println("radio disabled (boot/GPIO-only build)");
#endif
}

void loop() {
    // DIAGNOSTIC: drive the LED pin from a 1 Hz heartbeat as a UART-independent
    // liveness signal (the BDK wifi init appears to take over UART1, hiding our
    // Serial1 prints). The jig senses P6 via GPIO15, so 1 Hz EVT LED in MON
    // proves loop() runs even with no UART. apply_outputs() is suppressed here
    // so it doesn't fight the blink; monitor_rx still updates the controller.
    static uint32_t last_hb = 0;
    static bool hb = false;
    if (millis() - last_hb >= 1000) {
        last_hb = millis();
        hb = !hb;
        Serial1.printf("[hb] up=%lums led_pin=%d\r\n", (unsigned long)millis(), hb);
        digitalWrite(GPIO_LED, hb ? HIGH : LOW);
    }

    // Minimal bench console on UART1: single-char commands.
    while (Serial1.available()) {
        const char c = (char)Serial1.read();
        switch (c) {
            case 's': {  // stats
                const uint32_t now = millis();
                Serial1.printf("[stat] up=%lums frames=%lu ours=%lu relay_pkts=%lu led_pkts=%lu "
                               "relay=%d led=%d id=%u\r\n",
                               (unsigned long)now, (unsigned long)rx_frames,
                               (unsigned long)rx_ours, (unsigned long)rx_relay_pkts,
                               (unsigned long)rx_led_pkts,
                               (int)controller_relay_out(&ctrl, now),
                               (int)controller_led_out(&ctrl, now), my_id);
                break;
            }
            case 'd':  // hexdump next valid protocol frame
                dump_next = true;
                Serial1.println("[dump armed]");
                break;
            case '0' ... '9':  // quick id switch for bench tests: id = digit
                my_id = (uint8_t)(c - '0');
                Serial1.printf("[id=%u]\r\n", my_id);
                break;
            default:
                break;
        }
    }

    delay(10);
}
