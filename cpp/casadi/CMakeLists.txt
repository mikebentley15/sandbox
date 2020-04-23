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

function(add_app name)
  add_executable(${name} apps/${name}.cpp)
  target_link_libraries(${name} casadi)
  set_property(TARGET ${name} PROPERTY CXX_STANDARD 11)
endfunction(add_app)

add_app(ex01)
add_app(ex02)
add_app(ex03)
add_app(ex04)
add_app(nlpsol-introspect)
add_app(book-ex-1.1)
