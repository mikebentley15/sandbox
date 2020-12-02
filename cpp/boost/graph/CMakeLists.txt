cmake_minimum_required(VERSION 3.5)
project(boost_graph_sandbox)

find_package(Boost REQUIRED COMPONENTS graph)

add_executable(country-travel country-travel.cpp)
target_link_libraries(country-travel Boost::graph)
