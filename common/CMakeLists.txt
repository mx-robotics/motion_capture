
include_directories(include)
add_subdirectory(src/)

if(MOCAP_DEMOS_ENABLE)

add_executable(optitrack00 main00.cpp)
target_link_libraries(optitrack00 mocap)

add_executable(optitrack01 main01.cpp)
target_link_libraries(optitrack01 mocap)

endif(MOCAP_DEMOS_ENABLE)
