cmake_minimum_required(VERSION 3.25)
project(Camera LANGUAGES CXX CUDA)

# ================================================== Find Libraries ================================================== #

# OpenCV
find_package(OpenCV REQUIRED PATHS $ENV{OpenCV_DIR})

# Communication & YOLOv8
if (${CMAKE_BUILD_TYPE} MATCHES Debug)
    find_library(YOLOv8_LIB
                 NAMES YOLOv8_d
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/YOLOv8
                 PATH_SUFFIXES lib
                 )
    find_library(Engine_LIB
                 NAMES Engine_d
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/YOLOv8
                 PATH_SUFFIXES lib
                 )
else ()
    find_library(YOLOv8_LIB
                 NAMES YOLOv8
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/YOLOv8
                 PATH_SUFFIXES lib
                 )
    find_library(Engine_LIB
                 NAMES Engine
                 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/YOLOv8
                 PATH_SUFFIXES lib
                 )
endif ()

if (NOT YOLOv8_LIB)
    message(FATAL_ERROR "Found not YOLOv8.lib or YOLOv8_d.lib")
endif ()

if (NOT Engine_LIB)
    message(FATAL_ERROR "Found not Engine.lib or Engine_d.lib")
endif ()

set(YOLOv8_LIBS ${YOLOv8_LIB} ${Engine_LIB})

# Requires registration in the Path with the name ‘TensorRT_DIR’
set(TensorRT_ROOT $ENV{TensorRT_DIR})

if (NOT TensorRT_ROOT)
    message(WARNING "TensorRT path is incorrect.")
endif ()

message(STATUS "TensorRT Root: " ${TensorRT_ROOT})

# Find TensorRT include directory
find_path(TensorRT_INCLUDE_DIRS
          NAMES NvInfer.h
          HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} $ENV{Path}
          PATH_SUFFIXES include)

if (NOT TensorRT_INCLUDE_DIRS)
    message(FATAL_ERROR "* Can Not found TensorRT header")
endif ()

message(STATUS "Found TensorRT header")
message(STATUS "Tensor Include Root: " ${TensorRT_INCLUDE_DIRS})
message("")

# Find TensorRT Library
find_library(TensorRT_LIBRARY
             NAMES nvinfer_10
             HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} $ENV{Path}
             PATH_SUFFIXES lib lib64 lib/x64)

if (NOT TensorRT_LIBRARY)
    message(FATAL_ERROR "Can Not found TensorRT libraries")
endif ()

message(STATUS "Found TensorRT libraries")
message(STATUS "Tensor libraries: " ${TensorRT_LIBRARY})
message("")

# Find ONNX parser library
find_library(TensorRT_ONNXPARSER_LIBRARY
             NAMES nvonnxparser_10
             HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} $ENV{Path}
             PATH_SUFFIXES lib lib64 lib/x64)

if (NOT TensorRT_ONNXPARSER_LIBRARY)
    message(FATAL_ERROR "Can Not found TensorRT ONNX parser libraries")
endif ()

message(STATUS "Found TensorRT parser libraries")
message(STATUS "TensorRT ONNX parser libraries: " ${TensorRT_ONNXPARSER_LIBRARY})
message("")

# Group TensorRT libraries
set(TensorRT_LIBRARIES
    ${TensorRT_LIBRARY}
    ${TensorRT_ONNXPARSER_LIBRARY})

# ================================================ Build Configuration =============================================== #

file(GLOB_RECURSE SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_link_libraries(${PROJECT_NAME} PUBLIC
                      LIDAR
                      ${CUDA_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${TensorRT_LIBRARIES}
                      ${YOLOv8_LIBS}
                      )
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CUDA_INCLUDE_DIR}
                           ${TensorRT_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/YOLOv8/include
                           ${CUDA_INCLUDE_DIRS}
                           )
set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/calibration.hpp"
)

