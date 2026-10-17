# fw.mk — bare-SDK app source list for rc-switch-firmware.
#
# Included by the SDK's application.mk in place of OpenBeken's OpenBeken.mk.
# OBK_DIR is passed on the make command line and points at this directory
# (rc-switch-firmware/fw). We add ONLY our app; the SDK core (wifi, lwIP, flash,
# FreeRTOS, bootloader-compatible image build) comes from the SDK's own
# beken_src.mk. No OpenBeken HAL / web / MQTT / drivers / Berry.

INCLUDES += -I$(OBK_DIR)/main
# mbedtls (for DFU2 SHA-256) + lwip sockets come from the SDK; add mbedtls inc.
INCLUDES += -I./beken378/func/mbedtls/mbedtls-2.27.0/include

# CFG_SOC_NAME 7 == bk7238; mirror the platform define the SDK headers expect.
CCFLAGS += -DPLATFORM_BEKEN -DPLATFORM_BK7238
# our glue.c defines strcpy; don't let GCC treat it as a builtin (self-ref).
CCFLAGS += -fno-builtin-strcpy
# build id = sha256(version)[:8]; change USER_SW_VER to mint a new build id.
CCFLAGS += -DRCSW_VERSION=\"$(USER_SW_VER)\"

APP_C += $(OBK_DIR)/main/protocol.c
APP_C += $(OBK_DIR)/main/controller.c
APP_C += $(OBK_DIR)/main/user_main.c
APP_C += $(OBK_DIR)/main/glue.c
APP_C += $(OBK_DIR)/main/dfu2.c
APP_C += $(OBK_DIR)/main/settings.c

# The SDK's weak BLE stubs (ble_ctrl, ble_coex_pta_is_on, ...). BLE is off
# (CFG_SUPPORT_BLE=0), but the prebuilt librwnx/libcal still reference these;
# OpenBeken.mk compiles this file and we replaced OpenBeken.mk.
APP_C += ./fixes/blank.c
