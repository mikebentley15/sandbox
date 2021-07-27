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
  set_property(TARGET WithComponents_${subtarget} PROPERTY
    VERSION ${WithComponents_VERSION})
  set_property(TARGET WithComponents_${subtarget} PROPERTY
    SOVERSION ${WithComponents_MAJOR_VERSION})
  set_property(TARGET WithComponents_${subtarget} PROPERTY
    INTERFACE_WithComponents_${subtarget}_MAJOR_VERSION
      ${WithComponents_MAJOR_VERSION})
  set_property(TARGET WithComponents_${subtarget} APPEND PROPERTY
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
add_library(WithComponents_Base STATIC
  ${Base_HEADERS}
  ${SRC_DIR}/Base/A.cpp
  ${SRC_DIR}/Base/B.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/WithComponentsConfig.cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/WithComponents_BaseConfig.cmake
)
add_library(WithComponents::Base ALIAS WithComponents_Base)
target_link_libraries(WithComponents_Base PUBLIC Boost::filesystem)
target_include_directories(WithComponents_Base INTERFACE $<BUILD_INTERFACE:${SRC_DIR}>)
set_property(TARGET WithComponents_Base PROPERTY EXPORT_NAME Base)
set_version(Base)

set(Email_HEADERS ${SRC_DIR}/Email/Email.h)
install(
  FILES ${Email_HEADERS}
  DESTINATION ${HEADER_DEST}/Email
  COMPONENT Email
)
add_library(WithComponents_Email STATIC
  ${Email_HEADERS}
  ${SRC_DIR}/Email/Email.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/WithComponents_EmailConfig.cmake
)
add_library(WithComponents::Email ALIAS WithComponents_Email)
target_link_libraries(WithComponents_Email PUBLIC WithComponents::Base)
target_include_directories(WithComponents_Email INTERFACE $<BUILD_INTERFACE:${SRC_DIR}>)
set_property(TARGET WithComponents_Email PROPERTY EXPORT_NAME Email)
set_version(Email)

set(Telephone_HEADERS ${SRC_DIR}/Telephone/Telephone.h)
install(
  FILES ${Telephone_HEADERS}
  DESTINATION ${HEADER_DEST}/Telephone
  COMPONENT Telephone
)
add_library(WithComponents_Telephone STATIC
  ${Telephone_HEADERS}
  ${SRC_DIR}/Telephone/Telephone.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/WithComponents_TelephoneConfig.cmake
)
add_library(WithComponents::Telephone ALIAS WithComponents_Telephone)
target_link_libraries(WithComponents_Telephone PUBLIC WithComponents::Base)
target_include_directories(WithComponents_Telephone INTERFACE $<BUILD_INTERFACE:${SRC_DIR}>)
set_property(TARGET WithComponents_Telephone PROPERTY EXPORT_NAME Telephone)
set_version(Telephone)

install(TARGETS WithComponents_Base
  EXPORT WithComponents_Base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${HEADER_DEST}
)
install(TARGETS WithComponents_Email
  EXPORT WithComponents_Email
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${HEADER_DEST}
)
install(TARGETS WithComponents_Telephone
  EXPORT WithComponents_Telephone
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

export(EXPORT WithComponents_Base
  FILE "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_Base.cmake"
  NAMESPACE WithComponents::
)
export(EXPORT WithComponents_Email
  FILE "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_Email.cmake"
  NAMESPACE WithComponents::
)
export(EXPORT WithComponents_Telephone
  FILE "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_Telephone.cmake"
  NAMESPACE WithComponents::
)
configure_file(cmake/WithComponentsConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfig.cmake"
  COPYONLY
)
configure_file(cmake/WithComponents_BaseConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_BaseConfig.cmake"
  COPYONLY
)
configure_file(cmake/WithComponents_EmailConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_EmailConfig.cmake"
  COPYONLY
)
configure_file(cmake/WithComponents_TelephoneConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_TelephoneConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/WithComponents)
install(EXPORT WithComponents_Base
  FILE WithComponents_Base.cmake
  NAMESPACE WithComponents::
  DESTINATION ${ConfigPackageLocation}
)
install(EXPORT WithComponents_Email
  FILE WithComponents_Email.cmake
  NAMESPACE WithComponents::
  DESTINATION ${ConfigPackageLocation}
)
install(EXPORT WithComponents_Telephone
  FILE WithComponents_Telephone.cmake
  NAMESPACE WithComponents::
  DESTINATION ${ConfigPackageLocation}
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_BaseConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_EmailConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponents_TelephoneConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/WithComponents/WithComponentsConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    WithComponents_Base
)
