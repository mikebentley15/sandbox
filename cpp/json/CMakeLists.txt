cmake_minimum_required(VERSION 3.5)
project(json_sandbox)

add_executable(unsigned unsigned.cpp)
target_include_directories(unsigned PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_property(TARGET unsigned PROPERTY CXX_STANDARD 17)

add_executable(empty_array empty_array.cpp)
target_include_directories(empty_array PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_property(TARGET empty_array PROPERTY CXX_STANDARD 17)
