# Two targets from one tree: ESP8266_RTOS_SDK for the deployed KAUF bulbs, and
# ESP-IDF v5 for the ESP32-C3 board. The SDKs use different component-registration
# APIs, so the branches cannot be merged -- only the taken branch is evaluated.

set(BULB_SRCS
    "app_main.c"
    "pwm_output.c"
    "pwm_schedule.c"
    "pwm_mix.c"
    "bulb_config.c"
    "sniffer.c"
    "protocol.c"
    "controller.c"
    "dfu2.c"
)
set(BULB_INCLUDES "." "port" "variants")

if(IDF_TARGET STREQUAL "esp8266")
    set(COMPONENT_SRCS ${BULB_SRCS} "port/pwm_timer_wdev.c")
    set(COMPONENT_ADD_INCLUDEDIRS ${BULB_INCLUDES})
    set(COMPONENT_REQUIRES
        esp8266
        nvs_flash
        app_update
        spi_flash
        tcpip_adapter
        esp_event
        lwip
        mbedtls
        pthread   # pthread_cond_* pulled in by libstdc++ static-init guards
    )
    register_component()
else()
    idf_component_register(
        SRCS ${BULB_SRCS} "port/pwm_timer_systimer.c"
        INCLUDE_DIRS ${BULB_INCLUDES}
        REQUIRES
            nvs_flash
            app_update
            spi_flash
            esp_netif
            esp_event
            esp_wifi
            esp_hw_support
            esp_timer
            driver
            lwip
            mbedtls
            hal
            soc
    )
endif()
