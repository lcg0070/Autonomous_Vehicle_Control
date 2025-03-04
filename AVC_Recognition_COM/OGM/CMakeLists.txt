cmake_minimum_required(VERSION 3.25)
project(OGM LANGUAGES CXX)

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
     )

set(PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/OGM_processing.hpp)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${Communication_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )
target_link_libraries(${PROJECT_NAME} PUBLIC
                      ${Communication_LIB}
                      Camera
                      LIDAR
                      )

set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}")