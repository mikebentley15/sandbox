cmake_minimum_required(VERSION 3.4..3.18)
project(cliparser LANGUAGES CXX)

find_package(pybind11 REQUIRED)

set(SANDBOX_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
set(CLIPARSER_SRC ${SANDBOX_ROOT}/cpp/cliparser/src)

pybind11_add_module(cliparser cliparser_bindings.cpp)
target_include_directories(cliparser PUBLIC ${CLIPARSER_SRC})

