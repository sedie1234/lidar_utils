cmake_minimum_required(VERSION 3.11)


set(LIDAR_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/utils")
set(CMAKE_CUDA_ARCHITECTURES 70 75 80 86 89)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CUDACXX /usr/local/cuda/bin/nvcc)

set(INFER 0)

project(lidar_utility LANGUAGES CXX CUDA)

enable_language(CUDA)
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 11)

if(INFER)
set(Torch_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/share/cmake/Torch")
add_definitions(-DINFER=1)
find_package(Torch REQUIRED)
endif()


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

if(CLUSTERING)
add_definitions(-DCLUSTERING=1)
else()	
add_definitions(-DCLUSTERING=0)
endif()

add_subdirectory(utils)
add_subdirectory(example)
