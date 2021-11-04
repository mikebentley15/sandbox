cmake_minimum_required(VERSION 3.0)

project(interception-homeswap LANGUAGES C)

add_executable(printevent printevent.c)
target_compile_options(printevent PRIVATE -Wall -Wextra)
install(TARGETS printevent RUNTIME DESTINATION bin)
