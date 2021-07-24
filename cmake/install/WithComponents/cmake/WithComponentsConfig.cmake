include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED COMPONENTS filesystem)
include("${CMAKE_CURRENT_LIST_DIR}/WithComponentsTargets.cmake")
message(STATUS "WithComponents version ${WithComponents_VERSION}")
