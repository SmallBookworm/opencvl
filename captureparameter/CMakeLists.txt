cmake_minimum_required(VERSION 3.8)
project(captureparameter)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
set(SOURCE_FILES main.cpp)
add_executable(captureparameter ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(captureparameter ${OpenCV_LIBS})
