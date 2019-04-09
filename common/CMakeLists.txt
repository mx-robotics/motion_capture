
include_directories(include)
add_subdirectory(src/)

if(MOCAP_DEMOS_ENABLE)

add_executable(motion_capture00 main00.cpp)
target_link_libraries(motion_capture00 motion_capture)

add_executable(motion_capture01 main01.cpp)
target_link_libraries(motion_capture01 motion_capture)

add_executable(motion_capture02 main02.cpp)
target_link_libraries(motion_capture02 motion_capture -lpthread )

endif(MOCAP_DEMOS_ENABLE)
