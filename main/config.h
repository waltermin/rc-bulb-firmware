// config.h — all firmware tunables in one place.
//
// Wire-format constants (OUI, protocol version, max entries) live in protocol.h
// so the parser stays host-testable; everything else that a builder/operator
// might want to change is here.

#ifndef BULB_CONFIG_H
#define BULB_CONFIG_H

// ---- identity ---------------------------------------------------------------

// Fallback id used only when NVS has no provisioned "bulb_id" key.
#define BULB_DEFAULT_ID 0

// ---- default / fallback color (u8 per channel) ------------------------------
// Applied at boot before any radio, and again whenever the fallback timeout
// elapses. Defaults to a warm-ish white.
#define DEFAULT_R 0
#define DEFAULT_G 0
#define DEFAULT_B 0
#define DEFAULT_WW 255
#define DEFAULT_CW 0

// ---- control / fallback -----------------------------------------------------

// Revert to the default color if no BulbEntry addressed to us is seen within
// this window.
#define FALLBACK_TIMEOUT_MS 15000

// Fixed 802.11 channel the base station broadcasts on and we sniff on.
#define WIFI_CHANNEL 1

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

#define PWM_FREQ_HZ 250
#define PWM_PERIOD_US (1000000 / PWM_FREQ_HZ)  // 4000 us at 250 Hz

// Fraction of full scale the LEDs are allowed to reach (stock caps at 80%).
#define PWM_MAX_POWER 0.80f

// Gamma applied to each u8 channel value -> duty. 2.8 matches ESPHome's default
// gamma_correct used by the stock firmware. Set to 1.0f to disable.
#define PWM_GAMMA 2.8f

// Per-channel phase offset in DEGREES (-180..180], mirroring the stock phase
// offsets (fractions of the period): red 0.25=90, green 0.833≈-60, blue
// 0.667≈-120, cold/warm white aligned at 0.
#define PWM_PHASE_CW 0.0f
#define PWM_PHASE_RED 90.0f
#define PWM_PHASE_GREEN -60.0f
#define PWM_PHASE_BLUE -120.0f
#define PWM_PHASE_WW 0.0f

// ---- DFU / OTA --------------------------------------------------------------

// Hard-coded AP the bulb joins when entering DFU mode.
#define DFU_AP_SSID "Recurse Light DFU"
#define DFU_AP_PASS "changeme123"

// DHCP client hostname is DFU_HOSTNAME_PREFIX + decimal id, e.g. rc_light_dfu_7.
#define DFU_HOSTNAME_PREFIX "rc_light_dfu_"

// TCP port the DFU firmware-push server listens on.
#define DFU_TCP_PORT 3333

// How long to wait for association + DHCP before giving up and rebooting.
#define DFU_CONNECT_TIMEOUT_MS 30000

// How long to wait for a pushing client to connect before giving up.
#define DFU_ACCEPT_TIMEOUT_MS 120000

// Magic in the DFU push header (little-endian u32). "RClh" = 0x686C4352.
#define DFU_OTA_MAGIC 0x686C4352u

// ---- optional manual rollback guard -----------------------------------------
// ESP8266_RTOS_SDK has no esp_ota_mark_app_valid_cancel_rollback(). When this is
// enabled we keep an NVS boot-counter: a freshly-flashed image must run healthy
// for DFU_BOOT_VALIDATE_MS or the bootloader is pointed back at the previous
// slot on the next boot. Comment out to disable.
#define DFU_ROLLBACK_GUARD 1
#define DFU_BOOT_VALIDATE_MS 20000
#define DFU_BOOT_FAIL_THRESHOLD 3  // reverts after this many un-validated boots

#endif  // BULB_CONFIG_H
