cmake_minimum_required(VERSION 3.1.0)

project(hello_world)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if (CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

add_executable(hello_world main.cpp)
target_link_libraries(hello_world Qt5::Core Qt5::SerialPort)
