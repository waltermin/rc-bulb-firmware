# ESP8266_RTOS_SDK / ESP-IDF component for the Lemtone solver, pulled in by the
# top-level CMakeLists only for devices with DEVICE_LEMTONE. Only the solver and its
# compiled-in model go into the firmware; test/ and web/ are host and wasm builds
# driven by build.py.
#
# Each device links its own calibration: DEVICE_LEMTONE_MODEL in devices/<device>/device.cmake
# names the generated model file (default lemtone_model_default.c, the bulb). Every model file
# defines LEMTONE_MODEL_DEFAULT, so exactly one is linked.
include("${CMAKE_CURRENT_LIST_DIR}/../devices/$ENV{RC_DEVICE}/device.cmake")
if(NOT DEVICE_LEMTONE_MODEL)
    set(DEVICE_LEMTONE_MODEL "lemtone_model_default.c")
endif()
if(IDF_TARGET STREQUAL "esp8266")
    set(COMPONENT_SRCS "lemtone.c" "${DEVICE_LEMTONE_MODEL}")
    set(COMPONENT_ADD_INCLUDEDIRS ".")
    register_component()
else()
    idf_component_register(SRCS "lemtone.c" "${DEVICE_LEMTONE_MODEL}" INCLUDE_DIRS ".")
endif()

# Every component that includes lemtone.h must agree on the model struct, hence PUBLIC.
if(DEVICE_LEMTONE_MAX_KNOTS)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC LEMTONE_MAX_KNOTS=${DEVICE_LEMTONE_MAX_KNOTS})
endif()

# The project builds at -Og (debug); the solver is pure soft-float math on the hot
# path, so optimise it regardless.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
