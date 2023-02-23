cmake_minimum_required(VERSION 3.15)

# hunter package manager
include("cmake/HunterGate.cmake")
HunterGate(
  URL "https://github.com/cpp-pm/hunter/archive/v0.23.297.tar.gz"
  SHA1 "3319fe6a3b08090df7df98dee75134d68e2ef5a3"
)

project(fmt_test CXX)

hunter_add_package(fmt)
find_package(fmt REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} fmt)
