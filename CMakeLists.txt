# Top-level project file for the ESP8266_RTOS_SDK (CMake / idf.py) build.
cmake_minimum_required(VERSION 3.5)

# The Lemtone solver lives in lemtone/ (a standalone module with its own host and
# wasm builds); pull it in as a component.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/lemtone")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bulb-firmware)
