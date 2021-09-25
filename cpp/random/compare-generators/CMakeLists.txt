cmake_minimum_required(VERSION 3.5)
project(compare-generators VERSION 1.0.0 LANGUAGES CXX)

add_executable(compare-generators
  compare-generators.cpp
  Timer.h
  ScopedTiming.h
)
set_property(TARGET compare-generators PROPERTY CXX_STANDARD 17)
