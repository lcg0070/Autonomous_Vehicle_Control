cmake_minimum_required(VERSION 3.25)
project(Logger LANGUAGES C)

#============================================ Configuration Library ===================================================#

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
     )

file(GLOB_RECURSE PUBLIC_HEADER
     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
     )

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC GPSINS Control Path_Planning Time)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}"
                      )

add_dependencies(${PROJECT_NAME} Control Time Path_Planning GPSINS)
