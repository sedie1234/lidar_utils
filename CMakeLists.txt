cmake_minimum_required(VERSION 3.11)

project(lidar_utility VERSION 1.0)

set(LIDAR_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/utils")

add_subdirectory(utils)
add_subdirectory(example)