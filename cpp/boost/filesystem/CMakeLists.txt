cmake_minimum_required(VERSION 3.5)
project(boost_filesystem_sandbox)

find_package(Boost REQUIRED COMPONENTS filesystem)

add_executable(absolute_path absolute_path.cpp)
target_link_libraries(absolute_path Boost::filesystem)
