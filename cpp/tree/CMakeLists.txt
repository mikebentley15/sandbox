cmake_minimum_required(VERSION 3.5)
project(cpp-trees LANGUAGES CXX)

find_package(Boost COMPONENTS REQUIRED
  stacktrace_basic
  stacktrace_addr2line
  stacktrace_noop
  )

add_executable(stackless-tree-traversal stackless-tree-traversal.cpp)
set_property(TARGET stackless-tree-traversal PROPERTY CXX_STANDARD 17)
target_include_directories(stackless-tree-traversal
  PUBLIC ${Boost_INCLUDE_DIRS}
  )
target_link_libraries(stackless-tree-traversal
  PUBLIC ${Boost_STACKTRACE_BASIC_LIBRARY}
  PUBLIC ${Boost_STACKTRACE_ADDR2LINE_LIBRARY}
  PUBLIC ${Boost_STACKTRACE_NOOP_LIBRARY}
  PUBLIC pthread dl
  )
