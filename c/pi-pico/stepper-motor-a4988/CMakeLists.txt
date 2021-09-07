cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule 
# note: this must happen before project()
include(../pico-sdk/pico_sdk_init.cmake)

project(stepper_motor LANGUAGES C CXX ASM)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(stepper_motor
  main.cpp
  A4988.h
)
set_property(TARGET stepper_motor PROPERTY CXX_STANDARD 17)
target_link_libraries(stepper_motor pico_stdlib)
pico_add_extra_outputs(stepper_motor)
pico_enable_stdio_usb(stepper_motor 1)
pico_enable_stdio_uart(stepper_motor 0)

