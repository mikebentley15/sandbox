add_executable(heartlight heartlight.cpp)
set_property(TARGET heartlight PROPERTY CXX_STANDARD 20)
target_link_libraries(heartlight PRIVATE pico_stdlib)
