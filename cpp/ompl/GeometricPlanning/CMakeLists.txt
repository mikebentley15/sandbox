cmake_minimum_required(VERSION 3.5)
project(GeometricPlanning VERSION 1.0.0 LANGUAGES CXX)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
link_directories("/usr/local/lib64")
include_directories("${EIGEN3_INCLUDE_DIR}")

add_executable(GeometricPlanning GeometricPlanning.cpp)
target_link_libraries(GeometricPlanning
  ompl
  #${Boost_FILESYSTEM_LIBRARY}
  #${Boost_SYSTEM_LIBRARY}
  #${Boost_PROGRAM_OPTIONS_LIBRARY}
  )
