cmake_minimum_required(VERSION 3.25)
project(AVC LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)

include_directories(lib/DAQhal)
add_library(DAQhal lib/DAQhal/DAQhal.c)
link_directories(lib/DAQhal)
target_link_libraries(DAQhal NIDAQmx)

link_libraries(DAQhal)

add_subdirectory(modeling)
