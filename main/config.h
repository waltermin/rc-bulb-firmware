// config.h — all firmware tunables in one place.
//
// Wire-format constants (OUI, protocol version, max entries) live in protocol.h
// so the parser stays host-testable; everything else that a builder/operator
// might want to change is here.
//
// Some of these values (bulb id, default color, fallback timeout, Wi-Fi channel,
// duty curve, gamma, DFU AP/password) are now runtime-configurable via the
// NVS-backed config store (bulb_config.*). For those, the #define here is the
// DEFAULT used when the key is absent from flash — read them through the
// bulb_config_get_*(CFG_*) getters at runtime, not by their macro name.

#ifndef BULB_CONFIG_H
#define BULB_CONFIG_H

// ---- identity ---------------------------------------------------------------

// Fallback id used only when NVS has no provisioned "bulb_id" key.
#define BULB_DEFAULT_ID 0

// ---- default / fallback color (normalized [0.0, 1.0] per channel) -----------
// Applied at boot before any radio, and again whenever the fallback timeout
// elapses. Defaults to a warm-ish white.
#define DEFAULT_R 0.0f
#define DEFAULT_G 0.0f
#define DEFAULT_B 0.0f
#define DEFAULT_WW 1.0f
#define DEFAULT_CW 0.0f

// ---- control / fallback -----------------------------------------------------

// Revert to the default color if no BulbEntry addressed to us is seen within
// this window.
#define FALLBACK_TIMEOUT_MS 15000

// Fixed 802.11 channel the base station broadcasts on and we sniff on.
#define WIFI_CHANNEL 11

// ---- PWM (match original Kauf bulb behavior) --------------------------------
// GPIO assignments come from the stock kauf-bulb.yaml.
#define PWM_GPIO_RED 4
#define PWM_GPIO_GREEN 12
#define PWM_GPIO_BLUE 14
#define PWM_GPIO_CW 5
#define PWM_GPIO_WW 13

// ---- TEST BOARD OPTION ------------------------------------------------------
// On the bench devboard, GPIO12 (green) and GPIO14 (blue) are wired to an I2C
// device, so those two channels must NOT be driven. With this set to 1 the PWM
// module simply omits the green + blue channels; their pins are left untouched
// (high-Z inputs after reset, safe for the I2C bus). Set to 0 for real bulbs.
#ifndef OMIT_I2C_PINS
#define OMIT_I2C_PINS 0
#endif

// The active channel set is derived from OMIT_I2C_PINS in pwm_output.c (channel
// indices are generated there, not hard-coded here).

#define PWM_FREQ_HZ 1000
#define PWM_PERIOD_US (1000000 / PWM_FREQ_HZ)  // 1000 us at 1000 Hz

// Fraction of full scale the LEDs are allowed to reach (stock caps at 80%).
#define PWM_MAX_POWER 0.80f

// Duty-response curve: how a normalized channel level [0,1] maps to a
// normalized duty [0,1], before the max-power cap and period scaling.
//   PWM_CURVE_GAMMA      — power law, duty = level ^ PWM_GAMMA (uses PWM_GAMMA
//                          below); 2.8 matches the stock firmware.
//   PWM_CURVE_PERCEPTUAL — CIE L* perceptual lightness (linear toe near black
//                          then a cubic ramp); no powf() at runtime.
#define PWM_CURVE_GAMMA 0
#define PWM_CURVE_PERCEPTUAL 1
#define PWM_DUTY_CURVE PWM_CURVE_PERCEPTUAL

// Gamma applied to each channel level -> duty when PWM_DUTY_CURVE is
// PWM_CURVE_GAMMA. 2.8 matches ESPHome's default gamma_correct used by the stock
// firmware. Set to 1.0f to disable.
#define PWM_GAMMA 2.8f

// Per-channel phase offset in DEGREES (-180..180], mirroring the stock phase
// offsets (fractions of the period): red 0.25=90, green 0.833≈-60, blue
// 0.667≈-120, cold/warm white aligned at 0.
#define PWM_PHASE_CW 0.0f
#define PWM_PHASE_RED 90.0f
#define PWM_PHASE_GREEN -60.0f
#define PWM_PHASE_BLUE -120.0f
#define PWM_PHASE_WW 0.0f

// ---- DFU2 / pull-based OTA --------------------------------------------------
// DFU2 is pull-based: on a 0x05 Dfu2Request beacon (id range + AP creds + server
// IP/port + target build id) the bulb joins the advertised AP, connects OUT to
// the server, and pulls the image into the inactive OTA slot. The AP credentials
// come from the packet, not from here (see dfu2.c).

// DHCP client hostname is DFU_HOSTNAME_PREFIX + decimal id, e.g. rc_light_dfu_7.
// Kept for human-readable DHCP leases / logs during an update.
#define DFU_HOSTNAME_PREFIX "rc_light_dfu_"

// Magic in the DFU2 transfer header and hello (little-endian u32). "RClh".
#define DFU_OTA_MAGIC 0x686C4352u
#define DFU2_HELLO_MAGIC 0x32756C52u   // "Rlu2" — bulb->server hello

// How long to wait for association + DHCP after joining the AP before giving up
// and rebooting (into the untouched current firmware).
#define DFU2_CONNECT_TIMEOUT_MS 30000

// After we have an IP, how long to keep retrying the TCP connect to the server
// (it may not be listening the instant we join).
#define DFU2_TCP_CONNECT_WINDOW_MS 30000

// Per-recv stall guard once the transfer is underway (SO_RCVTIMEO).
#define DFU2_RECV_TIMEOUT_MS 15000

// Legacy: these were the fixed push-DFU AP credentials. DFU2 takes the AP from
// the Dfu2Request packet, so they are unused by the update path now; they remain
// only as the defaults for the (settable) CFG_DFU_SSID/CFG_DFU_PASS config keys.
#define DFU_AP_SSID "Recurse Light DFU"
#define DFU_AP_PASS "changeme123"

// ---- optional manual rollback guard -----------------------------------------
// ESP8266_RTOS_SDK has no esp_ota_mark_app_valid_cancel_rollback(). When this is
// enabled we keep an NVS boot-counter: a freshly-flashed image must run healthy
// for DFU_BOOT_VALIDATE_MS or the bootloader is pointed back at the previous
// slot on the next boot. Comment out to disable.
#define DFU_ROLLBACK_GUARD 1
#define DFU_BOOT_VALIDATE_MS 20000
#define DFU_BOOT_FAIL_THRESHOLD 3  // reverts after this many un-validated boots

#endif  // BULB_CONFIG_H
