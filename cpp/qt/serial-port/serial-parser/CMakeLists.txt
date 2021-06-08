cmake_minimum_required(VERSION 3.1.0)

project(serial_port)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if (CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

add_executable(serial-parser
  main.cpp
  SerialCommunicator.h
  SerialCommunicator.cpp
  MessageParser.h
  MessageParser.cpp
  MessageGenerator.h
  MessageGenerator.cpp
  )
target_link_libraries(serial-parser
  Qt5::Core
  Qt5::SerialPort
  )
