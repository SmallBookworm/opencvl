cmake_minimum_required(VERSION 3.9)
project(realtime)

set(CMAKE_CXX_STANDARD 11)

add_executable(realtime main.cpp ringTracker.cpp ringTracker.h)