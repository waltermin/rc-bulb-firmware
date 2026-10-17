// BKJIG2 — ESP32-S3 harness for reading/writing the T1-2S-NL's BK7238 flash.
//
// Wiring:
//   GPIO4 = target power relay, active-low (LOW = powered, HIGH = off)
//   GPIO6 / GPIO7 = target UART1 RX/TX, orientation unknown at wiring time;
//                   DETECT finds it and saves it to NVS (SWAP / PINS override)
//   CEN   = optional (PIN_CEN), open-drain (released = run, LOW = held in reset);
//           not wired at the moment, so the catch power-cycles via the relay
//
// Host link: the S3's native USB (USB-Serial-JTAG, /dev/cu.usbmodem*). Opening
// the port does not reset the S3, so command state survives reconnects.
//
// Modes:
//   Command mode (default): newline-terminated text commands, see HELP.
//   Bridge mode: transparent USB <-> target UART until the exit magic arrives
//     on the USB side, then back to command mode (no restart).
//
// Catching the BootROM: FLASHMODE power-cycles the target and spams the BK
// LinkCheck command itself, straight after power-on, until the ROM answers.
// Only then does it enter the bridge, so the host's flasher (bk7231tools)
// finds a chip already parked in download mode. Power is never cycled once
// the bridge is up, so a flash write can't be interrupted by the jig.

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "airtest.h"
#include "dfu2srv.h"
#include "radio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "nvs_flash.h"

#define VERSION "BKJIG2 v1"

#define PIN_RELAY  GPIO_NUM_4
#define PIN_UART_A GPIO_NUM_6  // moved off GPIO5, whose pad reads stuck low
#define PIN_UART_B GPIO_NUM_7
#define PIN_CEN    GPIO_NUM_NC  // set to a GPIO once CEN is wired again
#define HAVE_CEN   (PIN_CEN >= 0)

#define RELAY_ON_LEVEL  0  // active-low relay: LOW = target powered
#define RELAY_OFF_LEVEL 1

#define BK_UART       UART_NUM_1
#define BK_BAUD       115200
#define BK_RX_BUF     16384  // a whole 4 KB read reply even at multi-Mbaud

#define POWER_OFF_MS      1000  // long enough for the target's caps to drain
#define CATCH_WINDOW_MS   1500  // spam LinkCheck this long after power-on
#define CATCH_ATTEMPTS    4
#define CEN_RESET_MS      50    // CEN low time for a reset

// BK BootROM LinkCheck command and its response (bk7231tools packets.py):
//   cmd:  01 E0 FC <len=1> <code=0x00>
//   resp: 04 0E <len=5> 01 E0 FC <code+1=0x01> <value=0x00>
static const uint8_t LINK_CHECK[] = {0x01, 0xE0, 0xFC, 0x01, 0x00};
static const uint8_t LINK_RESP[]  = {0x04, 0x0E, 0x05, 0x01, 0xE0, 0xFC, 0x01, 0x00};

// Exit-bridge magic on the USB->target stream (same as the old jig/bkjig.ino).
static const uint8_t BRIDGE_EXIT[] = {0xDE, 0xAD, 0xBE, 0xEF, 'B', 'K', 'J', 'I',
                                      'G', '-', 'E', 'X', 'I', 'T', '4', '2'};

static int  s_tx_pin = PIN_UART_A;  // ESP TX -> target RX
static int  s_rx_pin = PIN_UART_B;  // ESP RX <- target TX
static bool s_pins_known = false;   // set once DETECT (or PINS/SWAP) has run
static int  s_baud = BK_BAUD;
static bool s_powered = true;

// ---- host I/O (USB-Serial-JTAG) ----------------------------------------------

static void usb_write(const void *buf, size_t len) {
    const uint8_t *p = buf;
    while (len > 0) {
        int n = usb_serial_jtag_write_bytes(p, len, pdMS_TO_TICKS(50));
        if (n <= 0) return;  // host not reading: drop rather than wedge
        p += n;
        len -= (size_t)n;
    }
}

static void out(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof buf - 3) n = sizeof buf - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    usb_write(buf, (size_t)n);
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

// ---- target UART ----------------------------------------------------------------

static void uart_apply_pins(void) {
    uart_set_pin(BK_UART, s_tx_pin, s_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_input_enable(s_tx_pin);  // so SCOPE can read back what we drive
    // Keep the receive line idle-high if the target is unpowered or unwired.
    gpio_set_pull_mode(s_rx_pin, GPIO_PULLUP_ONLY);
    uart_flush_input(BK_UART);
}

static void uart_init(void) {
    uart_config_t uc = {
        .baud_rate = s_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(BK_UART, BK_RX_BUF, 0, 0, NULL, 0);
    uart_param_config(BK_UART, &uc);
    uart_apply_pins();
}

// Float both UART lines (input, pulled down). An idle-high TX into an unpowered
// target's RX1 back-powers the chip through its ESD diodes, so it never sees a
// clean power-on reset and the bootloader never runs.
static void uart_release_pins(void) {
    const int pins[2] = {s_tx_pin, s_rx_pin};
    for (int i = 0; i < 2; i++) {
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(pins[i], GPIO_PULLDOWN_ONLY);
    }
}

// ---- target power -------------------------------------------------------------

static void power_set(bool on) {
    if (on) {
        gpio_set_level(PIN_RELAY, RELAY_ON_LEVEL);
        uart_apply_pins();
    } else {
        uart_release_pins();
        gpio_set_level(PIN_RELAY, RELAY_OFF_LEVEL);
    }
    s_powered = on;
}

static void power_cycle(uint32_t off_ms) {
    power_set(false);
    vTaskDelay(pdMS_TO_TICKS(off_ms));
    power_set(true);
}

// ---- persisted pin order ----------------------------------------------------------

static void pins_load(void) {
    nvs_handle_t h;
    if (nvs_open("bkjig2", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t tx, rx;
    if (nvs_get_u8(h, "tx", &tx) == ESP_OK && nvs_get_u8(h, "rx", &rx) == ESP_OK &&
        ((tx == PIN_UART_A && rx == PIN_UART_B) || (tx == PIN_UART_B && rx == PIN_UART_A))) {
        s_tx_pin = tx;
        s_rx_pin = rx;
        s_pins_known = true;
    }
    nvs_close(h);
}

static void pins_save(void) {
    nvs_handle_t h;
    if (nvs_open("bkjig2", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "tx", (uint8_t)s_tx_pin);
    nvs_set_u8(h, "rx", (uint8_t)s_rx_pin);
    nvs_commit(h);
    nvs_close(h);
    s_pins_known = true;
}

static void pins_set(int tx, int rx) {
    s_tx_pin = tx;
    s_rx_pin = rx;
    uart_apply_pins();
}

// ---- BootROM catch ----------------------------------------------------------------

// Catch timing, tunable at runtime via CATCH so it can be swept without
// reflashing (session only; FLASHMODE and DETECT use the current values).
typedef struct {
    int      settle_ms;  // after power-on, keep the UART released this long; -1 = auto
                         // (wait for the target's TX1 to go idle-high, up to 500 ms)
    uint32_t window_ms;  // how long to keep sending LinkCheck
    uint32_t gap_ms;     // wait for a reply this long between LinkChecks
    uint32_t off_ms;     // power-off time before the power-on (relay mode only)
    bool     use_cen;    // reset via CEN (default) instead of a relay power cycle
} catch_params_t;

static catch_params_t s_catch = {
    .settle_ms = 0,  // send from the instant of power-on; linked on the T1-2S-NL
                     // (auto, waiting for TX1 to rise at ~166 ms, is too late)
    .window_ms = CATCH_WINDOW_MS,
    .gap_ms = 5,  // same cadence as bk7231tools' wait_for_link
    .off_ms = POWER_OFF_MS,
    .use_cen = HAVE_CEN,
};

static void cen_pulse(uint32_t ms) {
    if (!HAVE_CEN) return;
    gpio_set_level(PIN_CEN, 0);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(PIN_CEN, 1);  // open-drain: released, the module's pull-up runs it
}

typedef struct {
    bool caught;
    uint32_t rx_bytes;  // everything the target sent during the window (diagnostic)
    int32_t  high_ms;   // auto settle: ms from power-on until TX1 went high (-1 = never)
    uint32_t sent;      // LinkChecks sent
} catch_result_t;

static void busy_wait_ms(uint32_t ms) {
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < end) {
    }
}

// Power-cycle the target and send LinkCheck until the BootROM answers or the
// window closes. The UART stays released (no back-powering through RX1) until
// the target's supply is up. On success the ROM is parked in download mode and
// any extra responses (one per LinkCheck it saw) are drained so the host
// starts clean.
static catch_result_t bootrom_catch(void) {
    catch_result_t r = {.high_ms = -1};
    if (s_baud != BK_BAUD) {  // the bootloader always starts at 115200
        s_baud = BK_BAUD;
        uart_set_baudrate(BK_UART, BK_BAUD);
    }
    size_t match = 0;
    uint8_t buf[128];

    if (s_catch.use_cen) {
        // Power stays up and the UART stays attached; CEN gives a clean reset
        // with an exactly known release time.
        if (!s_powered) {
            power_set(true);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        uart_apply_pins();
        cen_pulse(CEN_RESET_MS);
        if (s_catch.settle_ms > 0) busy_wait_ms((uint32_t)s_catch.settle_ms);
        uart_flush_input(BK_UART);
        goto spam;
    }

    power_set(false);  // releases the UART pins too
    vTaskDelay(pdMS_TO_TICKS(s_catch.off_ms));

    // Relay on, UART still released: the rx pin is an input with pull-down.
    gpio_set_level(PIN_RELAY, RELAY_ON_LEVEL);
    s_powered = true;
    int64_t t_on = esp_timer_get_time();
    if (s_catch.settle_ms < 0) {
        while (esp_timer_get_time() - t_on < 500000) {
            if (gpio_get_level(s_rx_pin)) {
                r.high_ms = (int32_t)((esp_timer_get_time() - t_on) / 1000);
                break;
            }
        }
    } else {
        busy_wait_ms((uint32_t)s_catch.settle_ms);
    }
    uart_apply_pins();  // attaches the UART and flushes stale input

spam:;
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < (int64_t)s_catch.window_ms * 1000 && !r.caught) {
        uart_write_bytes(BK_UART, LINK_CHECK, sizeof LINK_CHECK);
        uart_wait_tx_done(BK_UART, pdMS_TO_TICKS(10));
        r.sent++;
        int64_t gap_end = esp_timer_get_time() + (int64_t)s_catch.gap_ms * 1000;
        while (esp_timer_get_time() < gap_end && !r.caught) {
            int n = uart_read_bytes(BK_UART, buf, sizeof buf, 0);
            for (int i = 0; i < n; i++) {
                r.rx_bytes++;
                match = (buf[i] == LINK_RESP[match]) ? match + 1 : (buf[i] == LINK_RESP[0]) ? 1 : 0;
                if (match == sizeof LINK_RESP) {
                    r.caught = true;
                    break;
                }
            }
        }
    }

    if (r.caught) {
        // Drain until the line has been quiet for 50 ms.
        int64_t quiet = esp_timer_get_time();
        while (esp_timer_get_time() - quiet < 50000) {
            int n = uart_read_bytes(BK_UART, buf, sizeof buf, pdMS_TO_TICKS(10));
            if (n > 0) quiet = esp_timer_get_time();
        }
    }
    return r;
}

static void report_catch(const char *what, const catch_result_t *r) {
    out("%s: %s (sent %lu, rx %lu bytes, tx1-high %ld ms)", what,
        r->caught ? "linked" : "no answer", (unsigned long)r->sent,
        (unsigned long)r->rx_bytes, (long)r->high_ms);
}

static bool catch_with_retries(void) {
    for (int i = 0; i < CATCH_ATTEMPTS; i++) {
        catch_result_t r = bootrom_catch();
        char what[24];
        snprintf(what, sizeof what, "CATCH attempt %d", i + 1);
        report_catch(what, &r);
        if (r.caught) return true;
    }
    return false;
}

// ---- bridge -----------------------------------------------------------------------

// bk7231tools' SetBaudRate command: 01 E0 FC <len=6> <0x0F> <baud u32 LE> <delay_ms>.
// The chip switches delay_ms after receiving it and replies at the new rate;
// the host switches at delay_ms/2. Follow it so the bridge keeps up.
#define SETBAUD_LEN 10
static const uint8_t SETBAUD_HDR[] = {0x01, 0xE0, 0xFC, 0x06, 0x0F};

static void bridge_loop(void) {
    uint8_t buf[512];
    size_t exit_match = 0;
    uint8_t win[SETBAUD_LEN] = {0};  // last bytes host -> target
    uint32_t pending_baud = 0, pending_delay = 0;
    for (;;) {
        bool idle = true;

        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, 0);
        if (n > 0) {
            idle = false;
            for (int i = 0; i < n; i++) {
                memmove(win, win + 1, SETBAUD_LEN - 1);
                win[SETBAUD_LEN - 1] = buf[i];
                if (!memcmp(win, SETBAUD_HDR, sizeof SETBAUD_HDR)) {
                    pending_baud = win[5] | (win[6] << 8) | (win[7] << 16) | ((uint32_t)win[8] << 24);
                    pending_delay = win[9];
                }
                exit_match = (buf[i] == BRIDGE_EXIT[exit_match]) ? exit_match + 1
                           : (buf[i] == BRIDGE_EXIT[0])          ? 1
                                                                 : 0;
                if (exit_match == sizeof BRIDGE_EXIT) {
                    // Forward what preceded the magic, never the magic itself.
                    int keep = i + 1 - (int)sizeof BRIDGE_EXIT;
                    if (keep > 0) uart_write_bytes(BK_UART, buf, (size_t)keep);
                    uart_wait_tx_done(BK_UART, pdMS_TO_TICKS(100));
                    if (s_baud != BK_BAUD) {  // the target reboots at 115200
                        s_baud = BK_BAUD;
                        uart_set_baudrate(BK_UART, BK_BAUD);
                    }
                    out("\r\nBRIDGE EXIT");
                    return;
                }
            }
            uart_write_bytes(BK_UART, buf, (size_t)n);
            if (pending_baud >= 9600 && pending_baud <= 6000000) {
                uart_wait_tx_done(BK_UART, pdMS_TO_TICKS(100));
                busy_wait_ms(pending_delay / 2);
                uart_set_baudrate(BK_UART, pending_baud);
                s_baud = (int)pending_baud;
                pending_baud = 0;
            }
        }

        n = uart_read_bytes(BK_UART, buf, sizeof buf, 0);
        if (n > 0) {
            idle = false;
            usb_write(buf, (size_t)n);
        }

        if (idle) vTaskDelay(1);
    }
}

// ---- logic scope ------------------------------------------------------------------

// SCOPE <ms> [cen|power|none] [lc]: record every level change on the UART pins for
// <ms> after the trigger (CEN pulse, relay power-on, or immediately), optionally
// sending LinkCheck every 5 ms on the current TX pin. Prints "t_us bits" lines
// where bit0 = PIN_UART_A, bit1 = PIN_UART_B. A diagnostic for wiring/boot behaviour.
#define SCOPE_MAX_EDGES 20000
static uint32_t s_edge_t[SCOPE_MAX_EDGES];
static uint8_t  s_edge_v[SCOPE_MAX_EDGES];

static inline uint8_t scope_sample(void) {
    uint32_t in = REG_READ(GPIO_IN_REG);
    return (uint8_t)(((in >> PIN_UART_A) & 1u) | (((in >> PIN_UART_B) & 1u) << 1));
}

static void cmd_scope(uint32_t ms, const char *trig, bool lc) {
    if (ms < 1 || ms > 3000) ms = 500;
    if (!strcasecmp(trig, "cen") && !HAVE_CEN) {
        out("ERR no CEN pin configured");
        return;
    }
    if (!strcasecmp(trig, "cen")) {
        uart_apply_pins();
        gpio_set_level(PIN_CEN, 0);
        vTaskDelay(pdMS_TO_TICKS(CEN_RESET_MS));
    } else if (!strcasecmp(trig, "power")) {
        power_set(false);
        vTaskDelay(pdMS_TO_TICKS(s_catch.off_ms));
    }

    size_t n = 0;
    bool overflow = false;
    uint8_t last = scope_sample();
    s_edge_t[n] = 0;
    s_edge_v[n++] = last;

    if (!strcasecmp(trig, "cen")) {
        gpio_set_level(PIN_CEN, 1);
    } else if (!strcasecmp(trig, "power")) {
        gpio_set_level(PIN_RELAY, RELAY_ON_LEVEL);
        s_powered = true;
        uart_apply_pins();
    }
    int64_t t0 = esp_timer_get_time();
    int64_t next_lc = t0;
    int64_t end = t0 + (int64_t)ms * 1000;
    for (int64_t now = t0; now < end; now = esp_timer_get_time()) {
        if (lc && now >= next_lc) {
            uart_tx_chars(BK_UART, (const char *)LINK_CHECK, sizeof LINK_CHECK);  // FIFO, no block
            next_lc = now + 5000;
        }
        uint8_t v = scope_sample();
        if (v != last) {
            last = v;
            if (n < SCOPE_MAX_EDGES) {
                s_edge_t[n] = (uint32_t)(now - t0);
                s_edge_v[n++] = v;
            } else {
                overflow = true;
            }
        }
    }
    uart_flush_input(BK_UART);

    out("SCOPE trig=%s ms=%lu lc=%d edges=%u%s (bit0=GPIO%d bit1=GPIO%d)", trig,
        (unsigned long)ms, lc, (unsigned)n, overflow ? " OVERFLOW" : "", PIN_UART_A, PIN_UART_B);
    char line[160];
    size_t len = 0;
    for (size_t i = 0; i < n; i++) {
        len += (size_t)snprintf(line + len, sizeof line - len, "%lu:%u ",
                                (unsigned long)s_edge_t[i], s_edge_v[i]);
        if (len > sizeof line - 24 || i == n - 1) {
            out("%s", line);
            len = 0;
        }
    }
    out("SCOPE END");
}

// ---- commands ---------------------------------------------------------------------

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static const char *reset_reason(void) {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_USB:       return "usb";
        default:                return "other";
    }
}

static void print_state(void) {
    out("%s tx=%d rx=%d pins=%s baud=%d power=%s reset=%s", VERSION, s_tx_pin, s_rx_pin,
        s_pins_known ? "saved" : "default", s_baud, s_powered ? "on" : "off", reset_reason());
}

static void cmd_detect(void) {
    const int orders[2][2] = {{PIN_UART_A, PIN_UART_B}, {PIN_UART_B, PIN_UART_A}};
    const int prev_tx = s_tx_pin, prev_rx = s_rx_pin;
    for (int pass = 0; pass < 2; pass++) {
        for (int o = 0; o < 2; o++) {
            pins_set(orders[o][0], orders[o][1]);
            catch_result_t r = bootrom_catch();
            char what[24];
            snprintf(what, sizeof what, "DETECT tx=%d rx=%d", s_tx_pin, s_rx_pin);
            report_catch(what, &r);
            if (r.caught) {
                pins_save();
                power_cycle(POWER_OFF_MS);  // let the target boot its app again
                out("DETECT OK tx=%d rx=%d (saved)", s_tx_pin, s_rx_pin);
                return;
            }
        }
    }
    pins_set(prev_tx, prev_rx);  // nothing answered: restore the previous order
    out("DETECT FAIL (check wiring, ground, and that the relay powers the target)");
}

static void handle_line(char *line) {
    char *argv[10] = {0};
    int argc = 0;
    for (char *tok = strtok(line, " \t"); tok && argc < 10; tok = strtok(NULL, " \t")) {
        argv[argc++] = tok;
    }
    if (argc == 0) return;
    const char *cmd = argv[0];

    if (!strcasecmp(cmd, "PING")) {
        out("PONG");
        print_state();
    } else if (!strcasecmp(cmd, "STATE")) {
        print_state();
    } else if (!strcasecmp(cmd, "POWER")) {
        if (argc >= 2 && !strcasecmp(argv[1], "ON")) {
            power_set(true);
        } else if (argc >= 2 && !strcasecmp(argv[1], "OFF")) {
            power_set(false);
        } else if (argc >= 2 && !strcasecmp(argv[1], "CYCLE")) {
            int ms = argc >= 3 ? atoi(argv[2]) : POWER_OFF_MS;
            if (ms < 50 || ms > 10000) ms = POWER_OFF_MS;
            power_cycle((uint32_t)ms);
        } else {
            out("ERR usage: POWER ON|OFF|CYCLE [off_ms]");
            return;
        }
        out("POWER %s OK", s_powered ? "on" : "off");
    } else if (!strcasecmp(cmd, "SWAP")) {
        pins_set(s_rx_pin, s_tx_pin);
        pins_save();
        out("SWAP OK tx=%d rx=%d (saved)", s_tx_pin, s_rx_pin);
    } else if (!strcasecmp(cmd, "PINS")) {
        int tx = argc >= 3 ? atoi(argv[1]) : -1, rx = argc >= 3 ? atoi(argv[2]) : -1;
        if (!((tx == PIN_UART_A && rx == PIN_UART_B) || (tx == PIN_UART_B && rx == PIN_UART_A))) {
            out("ERR usage: PINS <tx> <rx> with {tx,rx} = {6,7}");
            return;
        }
        pins_set(tx, rx);
        pins_save();
        out("PINS OK tx=%d rx=%d (saved)", s_tx_pin, s_rx_pin);
    } else if (!strcasecmp(cmd, "BAUD")) {
        int b = argc >= 2 ? atoi(argv[1]) : 0;
        if (b < 1200 || b > 2000000) {
            out("ERR usage: BAUD <1200..2000000>");
            return;
        }
        s_baud = b;
        uart_set_baudrate(BK_UART, (uint32_t)b);
        out("BAUD %d OK", s_baud);
    } else if (!strcasecmp(cmd, "DETECT")) {
        cmd_detect();
    } else if (!strcasecmp(cmd, "CATCH")) {
        // CATCH [settle_ms|auto] [window_ms] [gap_ms] [off_ms]: update the catch
        // timing (kept for FLASHMODE/DETECT), then park the target in the BootROM
        // without entering the bridge.
        if (argc >= 2) s_catch.settle_ms = !strcasecmp(argv[1], "auto") ? -1 : atoi(argv[1]);
        if (argc >= 3) s_catch.window_ms = (uint32_t)atoi(argv[2]);
        if (argc >= 4) s_catch.gap_ms = (uint32_t)atoi(argv[3]);
        if (argc >= 5) s_catch.off_ms = (uint32_t)atoi(argv[4]);
        out("CATCH mode=%s settle=%d window=%lu gap=%lu off=%lu",
            s_catch.use_cen ? "cen" : "power", s_catch.settle_ms,
            (unsigned long)s_catch.window_ms, (unsigned long)s_catch.gap_ms,
            (unsigned long)s_catch.off_ms);
        out(catch_with_retries() ? "CATCH OK" : "CATCH FAIL");
    } else if (!strcasecmp(cmd, "FLASHMODE")) {
        if (!catch_with_retries()) {
            out("FLASHMODE FAIL bootrom did not answer");
            return;
        }
        out("FLASHMODE OK ENTERING BRIDGE");
        bridge_loop();
    } else if (!strcasecmp(cmd, "BRIDGE")) {
        // Plain passthrough (e.g. to watch the target's app log); no power cycling.
        out("BRIDGE OK");
        bridge_loop();
    } else if (!strcasecmp(cmd, "DFU2")) {
        // DFU2 START <len> <build_id_hex16> [dev_type] [id_start] [bounds] [ssid] [pass] [channel]
        // DFU2 TARGET <id_start> [bounds]   (retarget the request beacons)
        // DFU2 STOP
        // len 0 = probe mode: targets are pinged and released, nothing is served.
        // Serves the .rbl at DFU2SRV_IMG_ADDR over a soft AP and advertises it
        // with 0x05 Dfu2Request beacons, like a real update rollout.
        if (argc >= 2 && !strcasecmp(argv[1], "STOP")) {
            dfu2srv_stop();
            out("DFU2 STOP OK");
            return;
        }
        if (argc >= 3 && !strcasecmp(argv[1], "TARGET")) {
            dfu2srv_target((uint8_t)atoi(argv[2]), argc >= 4 ? (uint8_t)atoi(argv[3]) : 0);
            out("DFU2 TARGET OK %d..%d", atoi(argv[2]), atoi(argv[2]) + (argc >= 4 ? atoi(argv[3]) : 0));
            return;
        }
        if (argc < 4 || strcasecmp(argv[1], "START") || strlen(argv[3]) != 16) {
            out("ERR usage: DFU2 START <len> <build_id_hex16> [dev_type] [id_start] [bounds] [ssid] [pass] | DFU2 STOP");
            return;
        }
        dfu2srv_params_t dp = {
            .img_len = (uint32_t)strtoul(argv[2], NULL, 0),
            .dev_type = argc >= 5 ? (uint8_t)strtoul(argv[4], NULL, 0) : 0x02,
            .id_start = argc >= 6 ? (uint8_t)atoi(argv[5]) : 5,
            .id_bounds = argc >= 7 ? (uint8_t)atoi(argv[6]) : 0,
            .ssid = argc >= 8 ? argv[7] : "RC-Update",
            .pass = argc >= 9 ? argv[8] : "rcupdate1",
            .channel = argc >= 10 ? (uint8_t)atoi(argv[9]) : 11,
        };
        for (int i = 0; i < 8; i++) {
            int hi = hex_nibble(argv[3][2 * i]), lo = hex_nibble(argv[3][2 * i + 1]);
            if (hi < 0 || lo < 0) {
                out("ERR build id must be 16 hex chars");
                return;
            }
            dp.build_id[i] = (uint8_t)(hi << 4 | lo);
        }
        if (dp.img_len > 0x400000) {
            out("ERR bad image length");
            return;
        }
        if (airtest_running() || airtest_holding()) {
            out("ERR AIRTEST or HOLD running (stop them first)");
            return;
        }
        dfu2srv_start(&dp, out);
    } else if (!strcasecmp(cmd, "SEND")) {
        // SEND <body hex> [channel]: transmit exactly ONE beacon carrying this
        // packet body (tag + payload), e.g. a single 0x40 RelaysUpdate.
        size_t hl = argc >= 2 ? strlen(argv[1]) : 0;
        uint8_t body[70];
        if (hl == 0 || hl % 2 || hl / 2 > sizeof body) {
            out("ERR usage: SEND <1..70 bytes hex> [channel]");
            return;
        }
        for (size_t i = 0; i < hl / 2; i++) {
            int hi = hex_nibble(argv[1][2 * i]), lo = hex_nibble(argv[1][2 * i + 1]);
            if (hi < 0 || lo < 0) {
                out("ERR body must be hex");
                return;
            }
            body[i] = (uint8_t)(hi << 4 | lo);
        }
        int ch = argc >= 3 ? atoi(argv[2]) : 4;
        if (ch < 1 || ch > 14 || dfu2srv_running()) {
            out(dfu2srv_running() ? "ERR DFU2 serving" : "ERR bad channel");
            return;
        }
        radio_start_sta((uint8_t)ch);
        int e = radio_send_beacon(body, hl / 2);
        out("SEND %s %u bytes ch=%d (err %d)", e == 0 ? "OK" : "FAIL", (unsigned)(hl / 2), ch, e);
    } else if (!strcasecmp(cmd, "HOLD")) {
        // HOLD RELAY|LED <bitfield hex64> [channel] | HOLD RAW <slot> <body hex> [channel]
        // HOLD OFF | HOLD RELAY OFF | HOLD LED OFF | HOLD RAW OFF
        // Continuously broadcast a 0x40 (relay) / 0x41 (LED) state (bit N = id N),
        // or any packet body (e.g. 0x03 LightUpdateV2 for bulbs) in a raw slot.
        if (argc >= 2 && !strcasecmp(argv[1], "OFF")) {
            airtest_hold_off();
            out("HOLD OFF OK");
            return;
        }
        if (argc >= 3 && !strcasecmp(argv[2], "OFF")) {
            int what = !strcasecmp(argv[1], "RELAY") ? HOLD_CLEAR_RELAY
                     : !strcasecmp(argv[1], "LED")   ? HOLD_CLEAR_LED
                     : !strcasecmp(argv[1], "RAW")   ? HOLD_CLEAR_RAW : -1;
            if (what < 0) {
                out("ERR usage: HOLD RELAY|LED|RAW OFF");
                return;
            }
            airtest_hold_clear(what);
            out("HOLD %s OFF OK", argv[1]);
            return;
        }
        if (argc >= 4 && !strcasecmp(argv[1], "RAW")) {
            size_t hl = strlen(argv[3]);
            uint8_t body[70];
            if (hl % 2 || hl / 2 > sizeof body || hl == 0) {
                out("ERR raw body must be 1..70 bytes of hex");
                return;
            }
            for (size_t i = 0; i < hl / 2; i++) {
                int hi = hex_nibble(argv[3][2 * i]), lo = hex_nibble(argv[3][2 * i + 1]);
                if (hi < 0 || lo < 0) {
                    out("ERR raw body must be hex");
                    return;
                }
                body[i] = (uint8_t)(hi << 4 | lo);
            }
            int ch = argc >= 5 ? atoi(argv[4]) : 4;
            if (ch < 1 || ch > 14 || dfu2srv_running() ||
                !airtest_hold_raw(atoi(argv[2]), body, hl / 2, (uint8_t)ch)) {
                out("ERR bad slot (0..7) or channel, or DFU2 serving");
                return;
            }
            out("HOLD RAW %d OK %u bytes ch=%d", atoi(argv[2]), (unsigned)(hl / 2), ch);
            return;
        }
        bool led = argc >= 2 && !strcasecmp(argv[1], "LED");
        if (argc < 3 || (!led && strcasecmp(argv[1], "RELAY")) || strlen(argv[2]) != 64) {
            out("ERR usage: HOLD RELAY|LED <64 hex chars, byte N/8 bit N%%8> [channel] | HOLD OFF");
            return;
        }
        uint8_t bits[32];
        for (int i = 0; i < 32; i++) {
            int hi = hex_nibble(argv[2][2 * i]), lo = hex_nibble(argv[2][2 * i + 1]);
            if (hi < 0 || lo < 0) {
                out("ERR bitfield must be hex");
                return;
            }
            bits[i] = (uint8_t)(hi << 4 | lo);
        }
        int ch = argc >= 4 ? atoi(argv[3]) : 4;
        if (ch < 1 || ch > 14 || dfu2srv_running()) {
            out(dfu2srv_running() ? "ERR DFU2 serving" : "ERR bad channel");
            return;
        }
        airtest_hold(led, bits, (uint8_t)ch);
        out("HOLD %s OK ch=%d", led ? "LED" : "RELAY", ch);
    } else if (!strcasecmp(cmd, "AIRTEST")) {
        // AIRTEST <id> [channel] | AIRTEST STOP | AIRTEST STATUS
        // Relay/LED test pattern over the air (0x40/0x41) for a finished device;
        // runs in the background and stays silent so a flash can run meanwhile.
        if (argc >= 2 && !strcasecmp(argv[1], "STOP")) {
            airtest_stop();
            out("AIRTEST STOP OK");
        } else if (argc >= 2 && !strcasecmp(argv[1], "STATUS")) {
            out("AIRTEST %s", airtest_running() ? "running" : "idle");
        } else if (argc >= 2) {
            int id = atoi(argv[1]), ch = argc >= 3 ? atoi(argv[2]) : 4;
            if (id < 0 || id > 255 || ch < 1 || ch > 14) {
                out("ERR usage: AIRTEST <id 0..255> [channel 1..14] | STOP | STATUS");
            } else if (dfu2srv_running()) {
                out("ERR DFU2 serving (DFU2 STOP first)");
            } else if (!airtest_start((uint8_t)id, (uint8_t)ch)) {
                out("ERR AIRTEST already running");
            } else {
                out("AIRTEST started id=%d ch=%d (%lu ms)", id, ch,
                    (unsigned long)airtest_duration_ms());
            }
        } else {
            out("ERR usage: AIRTEST <id 0..255> [channel 1..14] | STOP | STATUS");
        }
    } else if (!strcasecmp(cmd, "SCOPE")) {
        cmd_scope(argc >= 2 ? (uint32_t)atoi(argv[1]) : 500, argc >= 3 ? argv[2] : "none",
                  argc >= 4 && !strcasecmp(argv[3], "lc"));
    } else if (!strcasecmp(cmd, "PIN")) {
        // PIN <6|7> HIGH|LOW|PU|PD|READ|UART: raw pad debug. HIGH/LOW drive the pin
        // as a plain GPIO; PU/PD make it an input with pull-up/down; UART restores
        // the UART routing. Every form reports the pad level read back.
        int pin = argc >= 2 ? atoi(argv[1]) : -1;
        const char *op = argc >= 3 ? argv[2] : "READ";
        if (pin != PIN_UART_A && pin != PIN_UART_B) {
            out("ERR usage: PIN <6|7> HIGH|LOW|PU|PD|READ|UART");
            return;
        }
        if (!strcasecmp(op, "HIGH") || !strcasecmp(op, "LOW")) {
            gpio_reset_pin(pin);
            gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
            gpio_set_level(pin, !strcasecmp(op, "HIGH"));
        } else if (!strcasecmp(op, "PU") || !strcasecmp(op, "PD")) {
            gpio_reset_pin(pin);
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(pin, !strcasecmp(op, "PU") ? GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY);
        } else if (!strcasecmp(op, "UART")) {
            uart_apply_pins();
        }
        esp_rom_delay_us(200);
        out("PIN %d %s -> level %d", pin, op, gpio_get_level(pin));
    } else if (!strcasecmp(cmd, "CEN") && !HAVE_CEN) {
        out("ERR no CEN pin configured");
    } else if (!strcasecmp(cmd, "CEN")) {
        int ms = argc >= 2 ? atoi(argv[1]) : CEN_RESET_MS;
        if (ms < 1 || ms > 10000) ms = CEN_RESET_MS;
        cen_pulse((uint32_t)ms);
        out("CEN pulse %d ms OK", ms);
    } else if (!strcasecmp(cmd, "CATCHMODE")) {
        if (argc >= 2 && !strcasecmp(argv[1], "CEN")) {
            if (!HAVE_CEN) {
                out("ERR no CEN pin configured");
                return;
            }
            s_catch.use_cen = true;
        } else if (argc >= 2 && !strcasecmp(argv[1], "POWER")) {
            s_catch.use_cen = false;
        } else {
            out("ERR usage: CATCHMODE CEN|POWER");
            return;
        }
        out("CATCHMODE %s OK", s_catch.use_cen ? "cen" : "power");
    } else if (!strcasecmp(cmd, "HELP")) {
        out("PING STATE POWER ON|OFF|CYCLE [ms] SEND hex [ch] AIRTEST id [ch] HOLD RELAY|LED hex64 [ch]|OFF DFU2 START|STOP CEN [ms] CATCHMODE CEN|POWER DETECT SWAP PINS tx rx BAUD n CATCH [settle|auto] [window] [gap] [off] FLASHMODE BRIDGE HELP");
        out("bridge exit magic: DE AD BE EF 'BKJIG-EXIT42'");
    } else {
        out("ERR unknown command '%s' (try HELP)", cmd);
    }
}

void app_main(void) {
    // Drive the relay to "on" before enabling the output so the target never
    // sees a power blip from the jig booting.
    gpio_set_level(PIN_RELAY, RELAY_ON_LEVEL);
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_RELAY,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    s_powered = true;

    // CEN: open-drain, released (the module's own pull-up keeps it running).
    if (HAVE_CEN) {
        gpio_set_level(PIN_CEN, 1);
        gpio_config_t cen = {
            .pin_bit_mask = 1ULL << PIN_CEN,
            .mode = GPIO_MODE_OUTPUT_OD,
        };
        gpio_config(&cen);
    }  // UART pins are attached by uart_init() below

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    pins_load();

    usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    // The driver drops USB packets when its RX ring is full (no flow control),
    // and the host sends bk7231tools' 4 KB flash-write packets in one burst
    // while the UART drains at ~11.5 KB/s. 16 KB holds a whole packet.
    uc.rx_buffer_size = 16384;
    uc.tx_buffer_size = 4096;
    usb_serial_jtag_driver_install(&uc);

    uart_init();

    char line[256];  // HOLD RAW carries up to 140 hex chars
    size_t len = 0;
    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) continue;
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
        } else if (len < sizeof line - 1) {
            line[len++] = (char)c;
        }
    }
}
