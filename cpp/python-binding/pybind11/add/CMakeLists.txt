cmake_minimum_required(VERSION 3.4..3.18)
project(adder LANGUAGES CXX)

find_package(pybind11 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(add_library add.cpp)

pybind11_add_module(add_module bindings/add_bindings.cpp)
target_link_libraries(add_module PUBLIC add_library)
