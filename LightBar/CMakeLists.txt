cmake_minimum_required(VERSION 3.10)
project(LightBar)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
add_executable(LightBar main.cpp linesOption.cpp lineTest.cpp lineInfo.cpp lineInfo.h)
target_link_libraries(LightBar ${OpenCV_LIBS})