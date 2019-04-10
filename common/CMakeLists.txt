
include_directories(include)
add_subdirectory(src/)

if(MOCAP_DEMOS_ENABLE)

add_executable(motion_capture_demo main_demo.cpp)
target_link_libraries(motion_capture_demo motion_capture  -lpthread )

endif(MOCAP_DEMOS_ENABLE)
