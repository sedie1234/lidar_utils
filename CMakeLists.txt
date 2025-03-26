cmake_minimum_required(VERSION 3.11)

project(lidar_utility VERSION 1.0)

set(LIDAR_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/utils")

if(PANORAMA)
add_definitions(-DPANORAMA_=1)
else()
add_definitions(-DPANORAMA_=0)
endif()

if(CV_VIEW)
add_definitions(-DCV_VIEW=1)
else()
add_definitions(-DCV_VIEW=0)
endif()

add_subdirectory(utils)
add_subdirectory(example)
add_definitions(-UStatus -UMode)
