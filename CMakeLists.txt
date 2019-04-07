cmake_minimum_required(VERSION 2.6)
project(optitrack)

set(CMAKE_CXX_FLAGS "-std=c++11")
option(MOCAP_DEMOS_ENABLE "Builds MOCAP demos" ON)

add_subdirectory(common)

