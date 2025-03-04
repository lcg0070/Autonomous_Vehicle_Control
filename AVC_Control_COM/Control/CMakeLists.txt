cmake_minimum_required(VERSION 3.25)
project(Control LANGUAGES C)

#============================================ Configuration Library ===================================================#

find_library(NIDAQmx_LIB
             NAMES NIDAQmx
             HINTS ${CMAKE_CURRENT_SOURCE_DIR}/DAQHal
             PATH_SUFFIXES lib
)

if (NOT NIDAQmx_LIB)
    message(FATAL_ERROR "Not found NIDAQmx.lib.")
endif (NOT NIDAQmx_LIB)

if (${CMAKE_BUILD_TYPE} MATCHES Debug)
    find_library(DAQHal_LIB
                 NAMES DAQHal_d
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/DAQHal
                 PATH_SUFFIXES lib
    )
else ()
    find_library(DAQHal_LIB
                 NAMES DAQHal
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/DAQHal
                 PATH_SUFFIXES lib
    )
endif ()

if (NOT DAQHal_LIB)
    message(FATAL_ERROR "Not found DAQHal.lib.")
endif (NOT DAQHal_LIB)

file(GLOB_RECURSE SOURCE
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

file(GLOB_RECURSE PUBLIC_HEADER
     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/DAQHal/include
)
target_link_libraries(${PROJECT_NAME} PUBLIC Time ${DAQHal_LIB} ${NIDAQmx_LIB})
set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADER}"
)

add_dependencies(${PROJECT_NAME} Time)
