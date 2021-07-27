# Make sure dependent components are already loaded
if (NOT TARGET WithComponents::Base)
  set(${CMAKE_FIND_PACKAGE_NAME}_FOUND False)
  return()
endif()

# Include the CMake auto-generated file with targets and dependencies
include("${CMAKE_CURRENT_LIST_DIR}/${CMAKE_FIND_PACKAGE_NAME}.cmake")
