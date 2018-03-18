cmake_minimum_required(VERSION 3.9)
project(realsense)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ()


find_package(OpenCV REQUIRED)
set(DEPENDENCIES realsense2 ${OpenCV_LIBS})
add_executable(rs-imshow main.cpp)
target_link_libraries(rs-imshow ${DEPENDENCIES})
target_link_libraries(rs-imshow pthread)
set_target_properties(rs-imshow PROPERTIES
        FOLDER "Examples/OpenCV"
        )

install(
        TARGETS

        rs-imshow

        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
)