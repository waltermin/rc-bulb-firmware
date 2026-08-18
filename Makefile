# Legacy (GNU make) build for ESP8266_RTOS_SDK. Alternative to idf.py/CMake.
#   make            # build
#   make flash      # build + flash over serial
# Requires IDF_PATH to point at your ESP8266_RTOS_SDK checkout.
PROJECT_NAME := bulb-firmware

include $(IDF_PATH)/make/project.mk
