cmake_minimum_required(VERSION 3.25)
project(AVC_Control_COM LANGUAGES C VERSION 0.5.1)

set(CMAKE_C_STANDARD 17)

#=============================================== Set Sub-project ======================================================#

add_subdirectory(Control)
add_subdirectory(Time)
add_subdirectory(Path_Planning)
add_subdirectory(GPSINS)
add_subdirectory(Logger)

#========================================== Configuration main program ================================================#

add_executable(${PROJECT_NAME} main.c)
target_link_libraries(${PROJECT_NAME} PUBLIC
                      Control Time GPSINS Path_Planning Logger
                      )

add_dependencies(${PROJECT_NAME} Control Time GPSINS Path_Planning Logger)
