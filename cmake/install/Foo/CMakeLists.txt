cmake_minimum_required(VERSION 3.5)
project(Foo LANGUAGES CXX)

set(Foo_VERSION 1.0)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(FOO_HEADERS ${SRC_DIR}/A.h ${SRC_DIR}/B.h)
set(FOO_SOURCES ${SRC_DIR}/A.cpp ${SRC_DIR}/B.cpp)
add_library(Foo STATIC ${FOO_SOURCES})
target_link_libraries(Foo PUBLIC Boost::filesystem)
# I'm trying to find a way to include the correct things...
target_include_directories(Foo INTERFACE
  $<INSTALL_INTERFACE:include/Foo>
)

## # This generates a header called foo_export.h
## # foo_export.h defines a few macros that can be used in the project
## # such as
## #   class Foo_EXPORT ClaaName { ... };
## # The macros are:
## # - Foo_EXPORT
## # - FOO_EXPORT
## # - FOO_NO_EXPORT
## # - FOO_DEPRECATED
## # - FOO_DEPRECATED_EXPORT
## # - FOO_DEPRECATED_NO_EXPORT
## include(GenerateExportHeader)
## generate_export_header(Foo)
## list(APPEND FOO_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/foo_export.h")

set_property(TARGET Foo PROPERTY VERSION ${Foo_VERSION})
set_property(TARGET Foo PROPERTY SOVERSION 1)
set_property(TARGET Foo PROPERTY INTERFACE_Foo_MAJOR_VERSION 1)
set_property(TARGET Foo APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING Foo_MAJOR_VERSION)
install(FILES ${FOO_HEADERS} DESTINATION include/Foo COMPONENT Devel)
install(TARGETS Foo EXPORT FooTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# TODO: need a FooConfig.cmake

#
# Create a FooConfig.cmake file
#

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/Foo/FooConfigVersion.cmake"
  VERSION ${Foo_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT FooTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/Foo/FooTargets.cmake"
  NAMESPACE Foo::
)
configure_file(cmake/FooConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/Foo/FooConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/Foo)
install(EXPORT FooTargets
  FILE FooTargets.cmake
  NAMESPACE Foo::
  DESTINATION ${ConfigPackageLocation}
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/Foo/FooConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/Foo/FooConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
