cmake_minimum_required(VERSION 3.5)
project(boost_gzip_sandbox)

find_package(Boost REQUIRED COMPONENTS iostreams)

add_executable(cat-gunzip cat-gunzip.cpp)
target_link_libraries(cat-gunzip Boost::iostreams)

add_executable(cat-gzip cat-gzip.cpp)
target_link_libraries(cat-gzip Boost::iostreams)
