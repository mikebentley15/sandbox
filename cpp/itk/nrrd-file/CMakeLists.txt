cmake_minimum_required(VERSION 3.5)
project(nrrd-file LANGUAGES CXX)

find_package(ITK REQUIRED)

include(${ITK_USE_FILE})

add_executable(read read.cpp)
set_property(TARGET read PROPERTY CXX_STANDARD 17)
target_link_libraries(read ${ITK_LIBRARIES})
target_include_directories(read PRIVATE ${ITK_INCLUDE_DIRS})
install(TARGETS read DESTINATION lib/${PROJECT_NAME})

add_executable(iminfo iminfo.cpp)
set_property(TARGET iminfo PROPERTY CXX_STANDARD 17)
target_link_libraries(iminfo ${ITK_LIBRARIES})
target_include_directories(iminfo PRIVATE ${ITK_INCLUDE_DIRS})
install(TARGETS iminfo DESTINATION lib/${PROJECT_NAME})
