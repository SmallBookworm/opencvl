cmake_minimum_required(VERSION 3.8)
project(depthtrack)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
add_executable(depthtrack main.cpp ball_tracker.cpp ball_tracker.h)
target_link_libraries(depthtrack ${OpenCV_LIBS})
target_link_libraries(depthtrack pthread)