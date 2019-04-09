cmake_minimum_required(VERSION 2.6)
project(optitrack)

find_package (Boost REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals system chrono)
link_libraries(${Boost_LIBRARIES} rt)

set(CMAKE_CXX_FLAGS "-std=c++11")
option(MOCAP_DEMOS_ENABLE "Builds MOCAP demos" ON)

add_subdirectory(common)

