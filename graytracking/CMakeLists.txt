cmake_minimum_required(VERSION 3.8)
project(graytracking)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
set(SOURCE_FILES main.cpp)
add_executable(graytracking ${SOURCE_FILES})
target_link_libraries(graytracking ${OpenCV_LIBS})