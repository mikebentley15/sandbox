include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED COMPONENTS filesystem)
include("${CMAKE_CURRENT_LIST_DIR}/FooTargets.cmake")
message(STATUS "Foo version ${Foo_VERSION}")
