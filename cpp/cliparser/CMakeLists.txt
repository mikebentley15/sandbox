cmake_minimum_required(VERSION 3.5)
project (CliParser VERSION 1.0.0 LANGUAGES CXX)

set(SRC_DIR src)

add_library(CliParser OBJECT src/CliParser.h)
target_include_directories(CliParser PUBLIC src)
set_target_properties(CliParser PROPERTIES LINKER_LANGUAGE CXX)

#############
#   tests   #
#############
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(TEST_FOLDER tests)

add_executable(AllTests
  ${TEST_FOLDER}/FunctionalTests.cpp
  ${TEST_FOLDER}/UnitTests.cpp
  )
set_property(TARGET AllTests PROPERTY CXX_STANDARD 14)
#target_link_libraries(AllTests CliParser gtest gtest_main)
target_link_libraries(AllTests CliParser GTest::GTest GTest::Main)
add_test(AllTests AllTests)
