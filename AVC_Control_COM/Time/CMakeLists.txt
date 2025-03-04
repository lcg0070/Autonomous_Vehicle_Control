cmake_minimum_required(VERSION 3.25)
project(Time LANGUAGES C)

#============================================ Configuration Library ===================================================#

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
     )

file(GLOB_RECURSE PUBLIC_HEADER
     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
     )

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# coummunication

target_link_libraries(${PROJECT_NAME} PUBLIC Control)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}"
                      )
