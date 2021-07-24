cmake_minimum_required(VERSION 3.5)
project(Bar LANGUAGES CXX)

set(Bar_VERSION 1.0)

find_package(Foo REQUIRED)
find_package(WithComponents REQUIRED COMPONENTS Base Email)

add_executable(bar main.cpp)
target_link_libraries(bar PUBLIC
  Foo::Foo
  WithComponents::Base
  WithComponents::Email
)
