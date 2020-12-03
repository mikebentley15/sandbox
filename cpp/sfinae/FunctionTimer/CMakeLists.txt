cmake_minimum_required(VERSION 3.5)
project(FunctionTimer VERSION 1.0.0 LANGUAGES CXX)

add_executable(returns_void
  returns_void.cpp
  returns_void.h
)
set_property(TARGET returns_void PROPERTY CXX_STANDARD 17)

add_executable(time_functions
  FunctionTimer.h
  time_function_call.h
  time_functions.cpp
  returns_void.h
)
set_property(TARGET time_functions PROPERTY CXX_STANDARD 17)

