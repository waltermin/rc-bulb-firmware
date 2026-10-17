# ESP8266_RTOS_SDK / ESP-IDF component for the Lemtone solver, pulled in by the
# top-level CMakeLists only for devices with DEVICE_LEMTONE. Only the solver and its
# compiled-in model go into the firmware; test/ and web/ are host and wasm builds
# driven by build.py.
if(IDF_TARGET STREQUAL "esp8266")
    set(COMPONENT_SRCS "lemtone.c" "lemtone_model_default.c")
    set(COMPONENT_ADD_INCLUDEDIRS ".")
    register_component()
else()
    idf_component_register(SRCS "lemtone.c" "lemtone_model_default.c" INCLUDE_DIRS ".")
endif()

# The project builds at -Og (debug); the solver is pure soft-float math on the hot
# path, so optimise it regardless.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
