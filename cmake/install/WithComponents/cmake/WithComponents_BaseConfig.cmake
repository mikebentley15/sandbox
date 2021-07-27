# Forward on the REQUIRED and QUIET keywords sent to find_package()
set(_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS)
if (${CMAKE_FIND_PACKAGE_NAME}_FIND_REQUIRED)
  set(_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS REQUIRED)
endif()
if (${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
  set(_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS QUIET)
endif()

# Find dependencies needed by this component
include(CMakeFindDependencyMacro)
find_package(Boost
  ${_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS}
  COMPONENTS filesystem
)

# Include the CMake auto-generated file with targets and dependencies
include("${CMAKE_CURRENT_LIST_DIR}/${CMAKE_FIND_PACKAGE_NAME}.cmake")
