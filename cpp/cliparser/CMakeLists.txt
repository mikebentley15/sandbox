cmake_minimum_required(VERSION 3.5)
project (CliParser VERSION 1.0.0 LANGUAGES CXX)

set(SRC_DIR src)

add_library(CliParser OBJECT ${SRC_DIR}/CliParser.h)
target_include_directories(CliParser PUBLIC ${SRC_DIR})

#############
#   tests   #
#############
enable_testing()

set(TEST_FOLDER tests)

add_executable(AllTests
  ${TEST_FOLDER}/FunctionalTests.cpp
  ${TEST_FOLDER}/UnitTests.cpp
  )
set_property(TARGET AllTests PROPERTY CXX_STANDARD 14)
target_link_libraries(AllTests CliParser gtest gtest_main)
add_test(AllTests AllTests)
