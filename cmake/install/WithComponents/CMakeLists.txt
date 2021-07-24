cmake_minimum_required(VERSION 3.5)
project(WithComponents LANGUAGES CXX)

set(WithComponents_MAJOR_VERSION 1)
set(WithComponents_MINOR_VERSION 0)
set(WithComponents_VERSION
  ${WithComponents_MAJOR_VERSION}.${WithComponents_MINOR_VERSION}
)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(HEADER_DEST include/WithComponents)


function(set_version subtarget)
  set_property(TARGET ${subtarget} PROPERTY
    VERSION ${WithComponents_VERSION})
  set_property(TARGET ${subtarget} PROPERTY
    SOVERSION ${WithComponents_MAJOR_VERSION})
  set_property(TARGET ${subtarget} PROPERTY
    INTERFACE_WithComponents_${subtarget}_MAJOR_VERSION
      ${WithComponents_MAJOR_VERSION})
  set_property(TARGET ${subtarget} APPEND PROPERTY
    COMPATIBLE_INTERFACE_STRING WithComponents_${subtarget}_MAJOR_VERSION)
endfunction(set_version)


set(Base_HEADERS
  ${SRC_DIR}/Base/A.h
  ${SRC_DIR}/Base/B.h
)
install(
  FILES ${Base_HEADERS}
  DESTINATION ${HEADER_DEST}/Base
  COMPONENT Base
)
add_library(Base STATIC
  ${Base_HEADERS}
  ${SRC_DIR}/Base/A.cpp
  ${SRC_DIR}/Base/B.cpp
)
add_library(WithComponents::Base ALIAS Base)
target_link_libraries(Base PUBLIC Boost::filesystem)
target_include_directories(Base INTERFACE $<BUILD_INTERFACE:${SRC_DIR}>)
set_version(Base)

set(Email_HEADERS ${SRC_DIR}/Email/Email.h)
install(
  FILES ${Email_HEADERS}
  DESTINATION ${HEADER_DEST}/Email
  COMPONENT Email
)
add_library(Email STATIC ${Email_HEADERS} ${SRC_DIR}/Email/Email.cpp)
add_library(WithComponents::Email ALIAS Email)
target_link_libraries(Email PUBLIC WithComponents::Base)
target_include_directories(Email INTERFACE $<BUILD_INTERFACE:${SRC_DIR}>)
set_version(Email)

set(Telephone_HEADERS ${SRC_DIR}/Telephone/Telephone.h)
install(
  FILES ${Telephone_HEADERS}
  DESTINATION ${HEADER_DEST}/Telephone
  COMPONENT Telephone
)
add_library(Telephone STATIC
  ${Telephone_HEADERS}
  ${SRC_DIR}/Telephone/Telephone.cpp
)
add_library(WithComponents::Telephone ALIAS Telephone)
target_link_libraries(Telephone PUBLIC WithComponents::Base)
target_include_directories(Telephone INTERFACE $<BUILD_INTERFACE:${SRC_DIR}>)
set_version(Telephone)

install(TARGETS Base Email Telephone EXPORT WithComponentsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${HEADER_DEST}
)


#
# Create a WithComponentsConfig.cmake file
#

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfigVersion.cmake"
  VERSION ${WithComponents_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT WithComponentsTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsTargets.cmake"
  NAMESPACE WithComponents::
)
configure_file(cmake/WithComponentsConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/WithComponents)
install(EXPORT WithComponentsTargets
  FILE WithComponentsTargets.cmake
  NAMESPACE WithComponents::
  DESTINATION ${ConfigPackageLocation}
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
