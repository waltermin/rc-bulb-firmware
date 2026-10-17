# The firmware, shared by every device. What differs per device comes from
# devices/<device>/device.cmake; what differs per chip lives in port/<chip>/. The two
# SDKs use different component-registration APIs, so only the taken branch is
# evaluated.

set(RC_DEVICE "$ENV{RC_DEVICE}")
include("${CMAKE_CURRENT_LIST_DIR}/../devices/${RC_DEVICE}/device.cmake")

set(RC_SRCS
    "app_main.c"
    "pwm_output.c"
    "pwm_schedule.c"
    "pwm_mix.c"
    "bulb_config.c"
    "sniffer.c"
    "protocol.c"
    "controller.c"
    "dfu2.c"
    "selftest.c"
)
# Everything chip-specific: the PWM timer backend (port/pwm_timer.h is the seam).
file(GLOB RC_PORT_SRCS RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/port/${DEVICE_TARGET}/*.c")
list(APPEND RC_SRCS ${RC_PORT_SRCS})
set(RC_INCLUDES "." "port" "port/${DEVICE_TARGET}" "variants")
set(RC_LEMTONE "")
if(DEVICE_LEMTONE)
    set(RC_LEMTONE lemtone)
endif()

if(DEVICE_TARGET STREQUAL "esp8266")
    set(COMPONENT_SRCS ${RC_SRCS})
    set(COMPONENT_ADD_INCLUDEDIRS ${RC_INCLUDES})
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
        ${RC_LEMTONE}
    )
    register_component()
else()
    idf_component_register(
        SRCS ${RC_SRCS}
        INCLUDE_DIRS ${RC_INCLUDES}
        REQUIRES
            nvs_flash
            app_update
            spi_flash
            esp_netif
            esp_event
            esp_wifi
            esp_hw_support
            esp_timer
            esp_driver_gpio
            lwip
            mbedtls
            hal
            soc
            ${RC_LEMTONE}
    )
endif()

# The device's identity, for the C side (config.h documents each).
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    "RC_DEVICE_NAME=\"${RC_DEVICE}\""
    "PROTO_DEVICE_TYPE=${DEVICE_TYPE}"
    "LIGHT_VARIANT_${DEVICE_VARIANT}=1"
    "LEMTONE_ENABLED=${DEVICE_LEMTONE}"
)

# On-device self-test build: BULB_SELFTEST=1 in the environment (build.ps1 reconfigures
# every run). It also needs the Lemtone reference vectors from the host tests.
if("$ENV{BULB_SELFTEST}" STREQUAL "1")
    if(NOT DEVICE_LEMTONE)
        message(FATAL_ERROR "BULB_SELFTEST needs a device with the Lemtone solver (bulb)")
    endif()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE LEMTONE_SELFTEST=1)
    target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../lemtone/test")
    message(STATUS "rc firmware: LEMTONE_SELFTEST build")
endif()
