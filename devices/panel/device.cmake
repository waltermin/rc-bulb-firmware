# The panel light: ESP32-C3 board (RGB direct + tunable-white driver IC), built
# with ESP-IDF v6. See devices/bulb/device.cmake for what each setting means.
set(DEVICE_TARGET  esp32c3)
set(DEVICE_PROJECT panel-firmware)
set(DEVICE_TYPE    0x03)
set(DEVICE_VARIANT C3_RGB_CCT)
# No Lemtone model for this hardware yet: 0x07 entries are ignored.
set(DEVICE_LEMTONE 0)
