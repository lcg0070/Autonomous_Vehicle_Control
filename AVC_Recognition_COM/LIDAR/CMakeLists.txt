cmake_minimum_required(VERSION 3.25)
project(LIDAR LANGUAGES C)

set(CMAKE_C_STANDARD 17)

add_subdirectory(interface)

# ================================================ Build Configuration =============================================== #

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/interface/*.c
     )

set(PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/LIDAR_processing.h)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_link_libraries(${PROJECT_NAME} PUBLIC
        interface
        ${Communication_LIB}
        ws2_32
)
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${Communication_INCLUDE_DIRS}
                           )
set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}")
