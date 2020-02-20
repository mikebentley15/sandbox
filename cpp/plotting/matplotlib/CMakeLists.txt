cmake_minimum_required(VERSION 3.12)
project(matplotlib_examples LANGUAGES CXX)

find_package(Python3 COMPONENTS Development NumPy)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(SOURCES
  ex01.cpp
  ex02.cpp
  ex03.cpp
  ex04.cpp
  ex05.cpp
  ex06.cpp
  )

foreach(source ${SOURCES})
  get_filename_component(target ${source} NAME_WE)
  add_executable(${target} ${source})
  set_property(TARGET ${target} PROPERTY CXX_STANDARD 14)
  target_include_directories(${target} PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    3rdparty
    )
  target_link_libraries(${target} Python3::Python Python3::NumPy)
endforeach(source)
