// config.h — all firmware tunables in one place.
//
// Wire-format constants (OUI, packet tags, device types) live in protocol.h so
// the parser stays host-testable; everything a builder/operator might change is
// here. Values marked (KV) are runtime-configurable via the flash config store
// / 0x04 SetConfig — the #define is the default used when the key is absent.

#ifndef RELAY_CONFIG_H
#define RELAY_CONFIG_H

// ---- identity ---------------------------------------------------------------

// Fallback id used only when the config store has no provisioned id. (KV)
#define RELAY_DEFAULT_ID 0

// ---- defaults / fallback ----------------------------------------------------

// DEFAULT output states: applied at power-on before the radio comes up, and
// again whenever the corresponding fallback timer expires. (KV)
#define DEFAULT_RELAY 0
#define DEFAULT_LED 0

// Per-packet-type fallback window: revert an output to its DEFAULT if no
// packet of its type (0x40 relay / 0x41 LED) is heard within this. (KV)
#define FALLBACK_TIMEOUT_MS 15000

// Fixed 802.11 channel the base station broadcasts on and we sniff on. (KV)
#define WIFI_CHANNEL 11

// ---- hardware (Tuya T1-2S-NL) ------------------------------------------------

#define GPIO_RELAY 9     // P9, drives the load relay, active-high
#define GPIO_LED 6       // P6, status LED
#define GPIO_BUTTON 8    // P8, active-low; per spec the button has NO function.
                         // The pin is left as a pulled-up input and never read
                         // (kept here so nothing else claims the pin).

// Whether the status LED output is inverted at the pin (stock hardware lights
// the LED when P6 is high; set to 1 if a board revision inverts that).
#define LED_ACTIVE_LOW 0

// ---- DFU2 / pull-based OTA ----------------------------------------------------

// DHCP client hostname is DFU_HOSTNAME_PREFIX + decimal id.
#define DFU_HOSTNAME_PREFIX "rc_relay_dfu_"

// Magic values shared with the bulb ecosystem (little-endian u32).
#define DFU_OTA_MAGIC 0x686C4352u      // "RClh" — server->device header
#define DFU2_HELLO_MAGIC 0x32756C52u   // "Rlu2" — device->server hello

// How long to wait for association + DHCP after joining the AP before giving
// up and rebooting (into the untouched current firmware).
#define DFU2_CONNECT_TIMEOUT_MS 30000

// After we have an IP, how long to keep retrying the TCP connect.
#define DFU2_TCP_CONNECT_WINDOW_MS 30000

// Per-recv stall guard once the transfer is underway.
#define DFU2_RECV_TIMEOUT_MS 15000

// ---- config store keys (0x04 SetConfig / UART console) ------------------------
// Relay devices use the 0x02xx key namespace.

#define CFG_KEY_RESTART_COUNTER 0x0201  // internal
#define CFG_KEY_DEVICE_ID 0x0202
#define CFG_KEY_DEFAULT_RELAY 0x0203
#define CFG_KEY_DEFAULT_LED 0x0204
#define CFG_KEY_WIFI_CHANNEL 0x0205
#define CFG_KEY_FALLBACK_TIMEOUT_MS 0x0206

#endif  // RELAY_CONFIG_H
