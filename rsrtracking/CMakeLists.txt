cmake_minimum_required(VERSION 3.9)
project(rsrtracking)

set(CMAKE_CXX_STANDARD 11)



find_package(OpenCV REQUIRED)
set(DEPENDENCIES realsense2 ${OpenCV_LIBS})
add_executable(rsrtracking main.cpp ball_tracker.cpp cv-helpers.hpp ring_watcher.cpp ring_watcher.h)
target_link_libraries(rsrtracking ${DEPENDENCIES} pthread)
