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
##set_source_files_properties(main.cpp PROPERTIES
##  COMPILE_FLAGS "-Wall -Wextra"
##)
target_link_libraries(stepper_motor PRIVATE pico_stdlib)
pico_add_extra_outputs(stepper_motor)
pico_enable_stdio_usb(stepper_motor 1)
pico_enable_stdio_uart(stepper_motor 0)


get_target_property(stepper_motor_SOURCES stepper_motor SOURCES)
add_executable(stepper_motor_demo ${stepper_motor_SOURCES})
target_compile_definitions(stepper_motor_demo PRIVATE ARMLAB_DEMO)
target_link_libraries(stepper_motor_demo PRIVATE pico_stdlib)
pico_add_extra_outputs(stepper_motor_demo)
pico_enable_stdio_usb(stepper_motor_demo 1)
pico_enable_stdio_uart(stepper_motor_demo 0)
