cmake_minimum_required(VERSION 3.25)
project(Path_Planning LANGUAGES C)

#============================================ Configuration Library ===================================================#

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

if(NOT Communication_LIB)
    message(FATAL_ERROR "Not Fount communication.lib")
endif ()

message("Communication Library:" ${Communication_LIB})

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
     )

file(GLOB_RECURSE PUBLIC_HEADER
     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
     )

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/Communication/include
)
target_link_libraries(${PROJECT_NAME} PUBLIC GPSINS Control ws2_32 ${Communication_LIB})

set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}"
                      )

add_dependencies(${PROJECT_NAME} GPSINS Control)
