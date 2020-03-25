cmake_minimum_required(VERSION 3.1.0)

project(ControllerPipeline)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Core Gui REQUIRED)

add_executable(controller_pipeline
  src/Controller.cpp
  src/Controller.h
  src/FakeReader.cpp
  src/FakeReader.h
  src/LocalizingFilter.cpp
  src/LocalizingFilter.h
  src/Logger.cpp
  src/Logger.h
  src/Registrar.cpp
  src/Registrar.h
  src/main.cpp
)
target_link_libraries(controller_pipeline Qt5::Core Qt5::Gui)
