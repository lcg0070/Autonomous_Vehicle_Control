cmake_minimum_required(VERSION 3.25)
project(interface LANGUAGES C)

set(CMAKE_C_STANDARD 17)

# ================================================ Build Configuration =============================================== #

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/urg/*.c
     )

set(PUBLIC_HEADER
    ${CMAKE_CURRENT_SOURCE_DIR}/include/SensorLidar.h
    )

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/urg/include
                           )
set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}"
                      )
