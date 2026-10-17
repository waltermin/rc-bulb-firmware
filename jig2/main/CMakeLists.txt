idf_component_register(
    SRCS "main.c" "dfu2srv.c" "radio.c" "airtest.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_usb_serial_jtag esp_driver_uart esp_driver_gpio nvs_flash esp_timer esp_wifi esp_netif esp_event lwip mbedtls spi_flash
)
