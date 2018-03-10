cmake_minimum_required(VERSION 3.9)
project(watchplate)

set(CMAKE_CXX_STANDARD 11)

add_executable(watchplate main.cpp)
find_package(OpenCV REQUIRED)
target_link_libraries(watchplate ${OpenCV_LIBS})