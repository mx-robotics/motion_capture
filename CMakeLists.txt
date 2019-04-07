cmake_minimum_required(VERSION 2.6)
project(optitrack)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(include)
add_subdirectory(src/)

add_executable(optitrack main.cpp)
target_link_libraries(optitrack mocap)

add_executable(optitrack00 main00.cpp)
target_link_libraries(optitrack00 mocap)

install(TARGETS optitrack RUNTIME DESTINATION bin)
