cmake_minimum_required(VERSION 3.8)


set(CMAKE_CXX_STANDARD 11)

if (WIN32 AND NOT MINGW)
    if (NOT DEFINED CMAKE_DEBUG_POSTFIX)
        set(CMAKE_DEBUG_POSTFIX "d")
    endif ()
endif ()

IF (NOT DEFINED CMAKE_BUILD_TYPE)
    # No effect for multi-configuration generators (e.g. for Visual Studio)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF ()

project(depthtrack)

SET(MY_DIR ${libfreenect2_examples_SOURCE_DIR})
SET(DEPENDS_DIR "${MY_DIR}/../depends" CACHE STRING "Dependency directory")


# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR ${MY_DIR}/..)
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
LIST(APPEND CMAKE_MODULE_PATH ${freenect2_ROOT_DIR}/cmake_modules) # FindGLFW3.cmake

IF (TARGET freenect2)
    MESSAGE(STATUS "Using in-tree freenect2 target")
    SET(freenect2_LIBRARIES freenect2)
    SET(freenect2_DLLS ${LIBFREENECT2_DLLS})
ELSE ()
    FIND_PACKAGE(freenect2 REQUIRED)
    # Out-of-tree build will have to have DLLs manually copied.
ENDIF ()

INCLUDE_DIRECTORIES(
        ${freenect2_INCLUDE_DIR}
)

SET(Protonect_src
        protonect.cpp
        )

SET(Protonect_LIBRARIES
        ${freenect2_LIBRARIES}
        )

SET(Protonect_DLLS
        ${freenect2_DLLS}
        )

LIST(APPEND Protonect_src
        main.cpp ball_tracker.cpp
        )

ADD_EXECUTABLE(depthtrack
        ${Protonect_src}
        )

TARGET_LINK_LIBRARIES(depthtrack
        ${Protonect_LIBRARIES}
        )

find_package(OpenCV REQUIRED)
target_link_libraries(depthtrack ${OpenCV_LIBS})
target_link_libraries(depthtrack pthread)

IF (WIN32)
    INSTALL(TARGETS Protonect DESTINATION bin)
    LIST(REMOVE_DUPLICATES Protonect_DLLS)
    FOREACH (FILEI ${Protonect_DLLS})
        ADD_CUSTOM_COMMAND(TARGET Protonect POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FILEI} $<TARGET_FILE_DIR:Protonect>
                )
    ENDFOREACH (FILEI)
    INSTALL(FILES ${Protonect_DLLS} DESTINATION bin)
ENDIF ()