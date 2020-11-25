cmake_minimum_required(VERSION 3.5)
project(voxel-collision VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3 -ffast-math")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl 0.5 EXACT REQUIRED)

add_executable(voxel-collision-example
  main.cpp
  collides.cpp
  AbstractVoxelOctree.cpp
  AbstractVoxelOctree.h
  CTSparseVoxelObject.h
  CTVoxelObject.h
  CTVoxelOctree.h
  CTVoxelOctree.hxx
  CTVoxelOctreeWrap.h
  CTVoxelOctreeWrap.hxx
  DerivedVoxelOctree.h
  DerivedVoxelOctree.hxx
  OctomapWrap.h
  SparseVoxelObject
  VoxelObject.h
  VoxelOctree.cpp
  VoxelOctree.h
  #VoxelOctree.hxx
  detail/TreeNode.h
  detail/TreeNode.hxx
  )
set_property(TARGET voxel-collision-example PROPERTY CXX_STANDARD 17)
target_link_libraries(voxel-collision-example
  ${OCTOMAP_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${FCL_LIBRARIES}
  )
target_include_directories(voxel-collision-example PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  )
