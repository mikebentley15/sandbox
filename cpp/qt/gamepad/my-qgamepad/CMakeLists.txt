cmake_minimum_required(VERSION 3.6)

project(my-qgamepad)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

add_executable(gamepad-logger
  main.cpp
  gamepad/gamepad.c
  Gamepad.h
  )

target_link_libraries(gamepad-logger Qt5::Core ${UDEV_LIBRARIES})
target_include_directories(gamepad-logger PUBLIC ${UDEV_INCLUDE_DIRS})
set_property(TARGET gamepad-logger PROPERTY CXX_STANDARD 17)
