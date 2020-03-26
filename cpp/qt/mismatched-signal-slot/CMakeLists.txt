cmake_minimum_required(VERSION 3.1.0)

project(MismatchedSignalSlot)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core Gui REQUIRED)

add_executable(mismatched_signal_slot
  src/MessageTypes.h
  src/Receiver.h
  src/Sender.h
  src/main.cpp
)
set_property(TARGET mismatched_signal_slot PROPERTY CXX_STANDARD 14)
target_link_libraries(mismatched_signal_slot Qt5::Core Qt5::Gui)
