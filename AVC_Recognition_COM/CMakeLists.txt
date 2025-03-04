cmake_minimum_required(VERSION 3.25)
project(AVC_Recognition_COM LANGUAGES C CXX CUDA VERSION 0.1.2)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(LIDAR)
add_subdirectory(Camera)
add_subdirectory(OGM)

add_dependencies(Camera LIDAR)
add_dependencies(OGM Camera LIDAR)

# ================================================== Find Libraries ================================================== #

find_path(Communication_INCLUDE_DIRS
          NAMES communication.h
          HINTS ${CMAKE_CURRENT_SOURCE_DIR}/Communication
          PATH_SUFFIXES include)

if (NOT Communication_INCLUDE_DIRS)
    message(FATAL_ERROR "Can Not found Communication header")
endif ()

if (${CMAKE_BUILD_TYPE} MATCHES Debug)
    find_library(Communication_LIB
                 NAMES communication_d
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/Communication
                 PATH_SUFFIXES lib
                 )
else ()
    find_library(Communication_LIB
                 NAMES communication
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/Communication
                 PATH_SUFFIXES lib
                 )
endif ()

if (NOT Communication_LIB)
    message(FATAL_ERROR "Found not communication.lib or communication_d.lib")
endif ()

file(GLOB_RECURSE SOURCES
     ${PROJECT_SOURCE_DIR}/main.cpp
     )

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC
                      LIDAR
                      Camera
                      OGM
                      ws2_32
                      )
