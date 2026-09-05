# fw.mk — bare-SDK app source list for rc-switch-firmware.
#
# Included by the SDK's application.mk in place of OpenBeken's OpenBeken.mk.
# OBK_DIR is passed on the make command line and points at this directory
# (rc-switch-firmware/fw). We add ONLY our app; the SDK core (wifi, lwIP, flash,
# FreeRTOS, bootloader-compatible image build) comes from the SDK's own
# beken_src.mk. No OpenBeken HAL / web / MQTT / drivers / Berry.

INCLUDES += -I$(OBK_DIR)/main

# CFG_SOC_NAME 7 == bk7238; mirror the platform define the SDK headers expect.
CCFLAGS += -DPLATFORM_BEKEN -DPLATFORM_BK7238
# our glue.c defines strcpy; don't let GCC treat it as a builtin (self-ref).
CCFLAGS += -fno-builtin-strcpy

APP_C += $(OBK_DIR)/main/protocol.c
APP_C += $(OBK_DIR)/main/controller.c
APP_C += $(OBK_DIR)/main/user_main.c
APP_C += $(OBK_DIR)/main/glue.c
