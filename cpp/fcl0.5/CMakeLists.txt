cmake_minimum_required(VERSION 3.5)
project(fcl_sandbox LANGUAGES CXX)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message("CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")


find_package(fcl 0.5 EXACT REQUIRED)

add_executable(mesh_collision
  mesh_collision.cpp
  Mesh.h
  Mesh.cpp
  3rdparty/stl_reader.h
)
target_include_directories(mesh_collision PUBLIC ${FCL_INCLUDE_DIRS})
target_link_libraries(mesh_collision PUBLIC ${FCL_LIBRARIES})
set_property(TARGET mesh_collision PROPERTY CXX_STANDARD 17)

