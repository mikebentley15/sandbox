cmake_minimum_required(VERSION 3.5)
project(boost_thread_pool_sandbox)

find_package(Boost REQUIRED COMPONENTS asio)

add_executable(child_thread_pool child_thread_pool.cpp)
target_link_libraries(child_thread_pool Boost::asio)
