set(COMPONENT_SRCS
    "app_main.c"
    "pwm_output.c"
    "bulb_config.c"
    "sniffer.c"
    "protocol.c"
    "controller.c"
    "dfu2.c"
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
    pthread   # provides pthread_cond_* pulled in by libstdc++ static-init guards
)

register_component()
