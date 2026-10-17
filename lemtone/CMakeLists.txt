# ESP8266_RTOS_SDK component for the Lemtone solver (found via EXTRA_COMPONENT_DIRS in
# the top-level CMakeLists). Only the solver and its compiled-in model go into the
# firmware; test/ and web/ are host and wasm builds driven by build.py.
set(COMPONENT_SRCS
    "lemtone.c"
    "lemtone_model_default.c"
)
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()

# The project builds at -Og (debug); the solver is pure soft-float math on the hot
# path, so optimise it regardless.
component_compile_options(-O2)
