cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule or elsewhere
# note: this must happen before project()
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/find_pico_sdk.cmake")

project(pi-pico-sandbox LANGUAGES C CXX ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(gpio_timing)
add_subdirectory(stepper-motor-a4988)
