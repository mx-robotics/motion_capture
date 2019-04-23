
include_directories(include)
add_subdirectory(src/)


        

if(MOCAP_DEMOS_ENABLE)

add_executable(motion_capture_demo main_demo.cpp)
target_link_libraries(motion_capture_demo motion_capture  -lpthread )

install(TARGETS motion_capture_demo
        RUNTIME DESTINATION bin)
        
endif(MOCAP_DEMOS_ENABLE)


install(DIRECTORY include/motion_capture DESTINATION include
        FILES_MATCHING PATTERN "*.h")
