cmake_minimum_required(VERSION 3.9)
project(realtime)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
add_executable(realtime main.cpp ringTracker.cpp ringTracker.h)
target_link_libraries(realtime realsense2 ${OpenCV_LIBS} pthread)