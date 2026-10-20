# The panel light: ESP32-C3 board (RGB direct + tunable-white driver IC), built
# with ESP-IDF v6. See devices/bulb/device.cmake for what each setting means.
set(DEVICE_TARGET  esp32c3)
set(DEVICE_PROJECT panel-firmware)
set(DEVICE_TYPE    0x03)
set(DEVICE_VARIANT C3_RGB_CCT)
# Lemtone with the panel's own calibration (calibration/export_panel_model.py).
set(DEVICE_LEMTONE 1)
set(DEVICE_LEMTONE_MODEL lemtone_model_panel.c)
