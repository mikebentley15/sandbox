cmake_minimum_required(VERSION 3.5)
project(casadi_sandbox LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find_package(PythonInterp 3 REQUIRED)
# execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
#   "from site import getsitepackages; print(';'.join(getsitepackages()))"
#   OUTPUT_VARIABLE PYTHON_SITE_MODULES
#   OUTPUT_STRIP_TRAILING_WHITESPACE)
# #message("PYTHON_SITE_MODULES = ${PYTHON_SITE_MODULES}")
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PYTHON_SITE_MODULES}")
# #message("CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")

find_package(casadi 3.5 MODULE)

add_executable(ex01 apps/ex01.cpp)
target_link_libraries(ex01 PUBLIC casadi)
set_property(TARGET ex01 PROPERTY CXX_STANDARD 11)

add_executable(ex02 apps/ex02.cpp)
target_link_libraries(ex02 PUBLIC casadi)
set_property(TARGET ex02 PROPERTY CXX_STANDARD 11)

add_executable(ex03 apps/ex03.cpp)
target_link_libraries(ex03 PUBLIC casadi)
set_property(TARGET ex03 PROPERTY CXX_STANDARD 11)

add_executable(ex04 apps/ex04.cpp)
target_link_libraries(ex04 PUBLIC casadi)
set_property(TARGET ex04 PROPERTY CXX_STANDARD 11)
