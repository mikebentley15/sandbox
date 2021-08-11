# Much of this logic was copied from Qt5Config.cmake and much was copied from
# the book Professional CMake

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# Make sure at least one component was specified
if (NOT ${CMAKE_FIND_PACKAGE_NAME}_FIND_COMPONENTS)
  set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE
    "The ${CMAKE_FIND_PACKAGE_NAME} package requires at least one component"
  )
  set(${CMAKE_FIND_PACKAGE_NAME}_FOUND False)
  return()
endif()


# ======================================
# Handle dependencies between components
# ======================================

set(_${CMAKE_FIND_PACKAGE_NAME}_COMPS
    ${${CMAKE_FIND_PACKAGE_NAME}_FIND_COMPONENTS})
# All components depend on the Base component
if (NOT Base IN_LIST _${CMAKE_FIND_PACKAGE_NAME}_COMPS)
  list(INSERT _${CMAKE_FIND_PACKAGE_NAME}_COMPS 0 Base) # prepend
endif()
# If there were inter-component dependencies, they would be specified here.

# ======================================
# End of dependencies between components
# ======================================


# Report to the console the found framework version
if (NOT ${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
  message(STATUS
    "${CMAKE_FIND_PACKAGE_NAME} version ${${CMAKE_FIND_PACKAGE_NAME}_VERSION}"
  )
endif()

# Forward on the REQUIRED and QUIET keywords sent to find_package()
set(_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS)
if (${CMAKE_FIND_PACKAGE_NAME}_FIND_REQUIRED)
  set(_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS REQUIRED)
endif()
if (${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
  set(_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS QUIET)
endif()

# set an empty value for this - which indicates no errors finding components
set(_${CMAKE_FIND_PACKAGE_NAME}_NOTFOUND_MESSAGE)

# include separate cmake config files for only the specified components
foreach(module ${_${CMAKE_FIND_PACKAGE_NAME}_COMPS})
  find_package(${CMAKE_FIND_PACKAGE_NAME}_${module}
    ${_${CMAKE_FIND_PACKAGE_NAME}_EXTRA_ARGS}
    PATHS ${CMAKE_CURRENT_LIST_DIR}
  )
  if (NOT ${CMAKE_FIND_PACKAGE_NAME}_${module}_FOUND
      AND ${CMAKE_FIND_PACKAGE_NAME}_FIND_REQUIRED_${module}
  )
    set(_${CMAKE_FIND_PACKAGE_NAME}_NOTFOUND_MESSAGE
      "${_${CMAKE_FIND_PACKAGE_NAME}_NOTFOUND_MESSAGE}Failed to find ${CMAKE_FIND_PACKAGE_NAME} component \"${module}\"\n"
    )
  else()
    message(STATUS "${CMAKE_FIND_PACKAGE_NAME}: Found component ${module}")
  endif()
endforeach()

# if we had an error finding components
if (_${CMAKE_FIND_PACKAGE_NAME}_NOTFOUND_MESSAGE)
  set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE
    "${_${CMAKE_FIND_PACKAGE_NAME}_NOTFOUND_MESSAGE}"
  )
  set(${CMAKE_FIND_PACKAGE_NAME}_FOUND False)
endif()
