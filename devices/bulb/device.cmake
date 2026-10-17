# The Kauf RGBWW bulb: ESP8266, built with ESP8266_RTOS_SDK.
#
# A device is a product, not a chip: everything that differs between products is
# chosen here, and both the top-level CMakeLists and main/CMakeLists read it.
#   DEVICE_TARGET   SDK target (esp8266 -> ESP8266_RTOS_SDK, anything else -> ESP-IDF)
#   DEVICE_PROJECT  CMake project name = image file name = esp_app_desc.project_name
#   DEVICE_TYPE     0x05 Dfu2Request device_type this image answers (spec.txt)
#   DEVICE_VARIANT  board pin table in main/variants/ (LIGHT_VARIANT_<name>)
#   DEVICE_LEMTONE  1 = link the Lemtone solver and act on 0x07 LemtoneUpdate
set(DEVICE_TARGET  esp8266)
set(DEVICE_PROJECT bulb-firmware)   # deployed bulbs carry this name; keep it
set(DEVICE_TYPE    0x01)
set(DEVICE_VARIANT KAUF_RGBWW)
set(DEVICE_LEMTONE 1)
