cmake_minimum_required(VERSION 3.8)
project(hough_detector)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp detector.cpp detector.h)

find_package( OpenCV REQUIRED )
add_executable(hough_detector ${SOURCE_FILES})
target_link_libraries( hough_detector  ${OpenCV_LIBS} )
