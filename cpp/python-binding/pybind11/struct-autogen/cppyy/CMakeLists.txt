cmake_minimum_required(VERSION 3.4..3.18)
project(LANGUAGES CXX)

find_package(pybind11 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(struct_headers INTERFACE)
target_include_directories(struct_headers INTERFACE ../binder)

pybind11_add_module(mystructs
  expected.cpp
  #actual.cpp
  )
target_link_libraries(mystructs PRIVATE struct_headers)

