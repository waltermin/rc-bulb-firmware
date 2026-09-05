// glue.c — minimal shims the beken_freertos_sdk fork expects from the app.
//
// This SDK fork (as used by OpenBeken) has a couple of app-provided hooks baked
// into its core: a power-save level setter + weak PHY low-power stubs (normally
// supplied by OpenBeken's platforms/BK723x/ps.c), and CFG_GetOpenBekenHostName()
// which the SDK's lwip ethernetif.c calls for the DHCP hostname. We provide just
// these — none of OpenBeken's OS.

#include "include.h"
#include "wlan_ui_pub.h"

// ---- power save (from ps.c) -------------------------------------------------
BK_PS_LEVEL global_ps_level = 0;
int bk_wlan_power_save_set_level(BK_PS_LEVEL level) {
    // Relay switch stays always-on in monitor mode; no STA/MCU power-save.
    global_ps_level = level;
    return 0;
}

void __attribute__((weak)) phy_enter_11b_low_power(void) {}
void __attribute__((weak)) phy_exit_11b_low_power(void) {}

// ---- DHCP hostname (SDK lwip hook) -----------------------------------------
const char *CFG_GetOpenBekenHostName(void) {
    return "rc-switch";
}

// ---- strcpy: SDK builds with -nostdlib; LTO synthesizes a strcpy call that no
// SDK object provides. Supply a minimal one. -fno-builtin-strcpy (in fw.mk)
// keeps GCC from turning this definition into a self-referential builtin.
char *strcpy(char *dst, const char *src) {
    char *r = dst;
    while ((*dst++ = *src++) != '\0') { }
    return r;
}
