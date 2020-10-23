cmake_minimum_required(VERSION 3.5)
project(cpp-trees LANGUAGES CXX)

add_executable(stackless-tree-traversal stackless-tree-traversal.cpp)
set_property(TARGET stackless-tree-traversal PROPERTY CXX_STANDARD 17)
