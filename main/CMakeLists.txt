set(COMPONENT_SRCS
    "app_main.c"
    "pwm_output.c"
    "id_store.c"
    "sniffer.c"
    "protocol.c"
    "controller.c"
    "dfu.c"
)
set(COMPONENT_ADD_INCLUDEDIRS ".")

# Depend explicitly on the components we use so headers resolve under CMake.
set(COMPONENT_REQUIRES
    esp8266
    nvs_flash
    app_update
    spi_flash
    tcpip_adapter
    esp_event
    lwip
    mbedtls
)

register_component()
