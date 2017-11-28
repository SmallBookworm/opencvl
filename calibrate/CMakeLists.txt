cmake_minimum_required(VERSION 3.8)
project(calibrate)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
set(SOURCE_FILES main.cpp)
add_executable(calibrate ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(calibrate ${OpenCV_LIBS})