cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule 
# note: this must happen before project()
include(../pico-sdk/pico_sdk_init.cmake)

project(gpio_timing)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(gpio_timing gpio_timing.c)
target_link_libraries(gpio_timing pico_stdlib)
pico_add_extra_outputs(gpio_timing)
pico_enable_stdio_usb(gpio_timing 1)
pico_enable_stdio_uart(gpio_timing 0)
