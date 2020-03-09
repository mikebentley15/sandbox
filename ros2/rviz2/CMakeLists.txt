cmake_minimum_required(VERSION 3.5)
project(sandbox_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(basic_shapes src/basic_shapes.cpp)
ament_target_dependencies(basic_shapes rclcpp visualization_msgs)
install(TARGETS basic_shapes DESTINATION lib/${PROJECT_NAME})

add_executable(points_and_lines src/points_and_lines.cpp)
ament_target_dependencies(points_and_lines rclcpp visualization_msgs)
install(TARGETS points_and_lines DESTINATION lib/${PROJECT_NAME})

# not yet working
# add_executable(interactive_simple_marker src/interactive_simple_marker.cpp)
# ament_target_dependencies(interactive_simple_marker rclcpp interactive_markers)
# install(TARGETS interactive_simple_marker DESTINATION lib/${PROJECT_NAME})

# add_executable(view_tendon src/view_tendon.cpp)
# install(TARGETS view_tendon DESTINATION lib/${PROJECT_NAME})

ament_package()
