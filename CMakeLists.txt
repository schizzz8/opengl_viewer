cmake_minimum_required(VERSION 2.8.11)
project(opengl_viewer)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#configure_file(configuration/root_directory.h.in root_directory.h)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_BINARY_DIR}
)

find_package(glfw3 REQUIRED)
if(glfw3_FOUND)
    MESSAGE(STATUS "GLFW3 found!")
    MESSAGE(STATUS "GLFW3 include dirs: " ${GLFW3_INCLUDE_DIR})
    MESSAGE(STATUS "GLFW3 libs: " ${GLFW3_LIBRARY})
endif(glfw3_FOUND)
include_directories(${GLFW3_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
if (NOT OPENGL_FOUND)
    message (FATAL_ERROR "OpenGL not found!")
    return()
endif ()
include_directories(${OPENGL_INCLUDE_DIR})

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(assimp REQUIRED)
#if(assimp_FOUND)
#    MESSAGE(STATUS "ASSIMP found!")
#    MESSAGE(STATUS "ASSIMP include dirs: " ${ASSIMP_INCLUDE_DIRS})
#    MESSAGE(STATUS "ASSIMP libs: " ${ASSIMP_LIBRARIES})
#endif(assimp_FOUND)
#include_directories(${ASSIMP_INCLUDE_DIRS})

add_subdirectory(src)
