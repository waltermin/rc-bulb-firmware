# One tree, several devices. Pick one with -DDEVICE=<name> (or RC_DEVICE in the
# environment); build.ps1 -Device / build.sh <device> do this for you. Each device
# lives in devices/<name>/: device.cmake (what it is), sdkconfig.defaults and
# partitions.csv. Shared code is in main/ and lemtone/, chip code in main/port/<chip>/.
cmake_minimum_required(VERSION 3.16)

if(NOT DEVICE)
    set(DEVICE "$ENV{RC_DEVICE}")
endif()
file(GLOB _devices RELATIVE "${CMAKE_CURRENT_LIST_DIR}/devices" "${CMAKE_CURRENT_LIST_DIR}/devices/*")
if(NOT DEVICE OR NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/devices/${DEVICE}/device.cmake")
    message(FATAL_ERROR "set DEVICE to one of: ${_devices} (e.g. -DDEVICE=bulb)")
endif()
set(DEVICE_DIR "${CMAKE_CURRENT_LIST_DIR}/devices/${DEVICE}")
include("${DEVICE_DIR}/device.cmake")

if(DEFINED IDF_TARGET AND NOT IDF_TARGET STREQUAL DEVICE_TARGET)
    message(FATAL_ERROR "device '${DEVICE}' builds for ${DEVICE_TARGET}, not IDF_TARGET=${IDF_TARGET}")
endif()
set(IDF_TARGET "${DEVICE_TARGET}")
set(ENV{IDF_TARGET} "${DEVICE_TARGET}")

# Per-device config so the two builds never share state.
set(SDKCONFIG_DEFAULTS "${DEVICE_DIR}/sdkconfig.defaults")
if(NOT SDKCONFIG)
    set(SDKCONFIG "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.${DEVICE}")
endif()
# ESP-IDF creates a missing sdkconfig from the defaults; ESP8266_RTOS_SDK insists it
# already exists, so seed it the same way on first configure.
if(NOT EXISTS "${SDKCONFIG}")
    configure_file("${SDKCONFIG_DEFAULTS}" "${SDKCONFIG}" COPYONLY)
endif()

# Components (main/, lemtone/) re-read device.cmake; the SDKs evaluate component
# CMakeLists in child CMake processes, which inherit the environment.
set(ENV{RC_DEVICE} "${DEVICE}")

if(DEVICE_LEMTONE)
    # The Lemtone solver (a standalone module with its own host and wasm builds).
    set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/lemtone")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
message(STATUS "rc firmware: device ${DEVICE} (${DEVICE_TARGET}, type ${DEVICE_TYPE}, variant ${DEVICE_VARIANT}, lemtone ${DEVICE_LEMTONE})")
project(${DEVICE_PROJECT})
