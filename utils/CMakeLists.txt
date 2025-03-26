
# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# opencv library
find_package(OpenCV 4.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# OusterSDK
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/OusterSDK" ${CMAKE_PREFIX_PATH})
find_package(OusterSDK REQUIRED COMPONENTS Static)

# eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_library(lidar_util STATIC
    lidar_util.cpp
    vis_util.cpp
)

target_link_libraries(lidar_util
    OusterSDK::ouster_client
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

add_definitions(-UStatus -UMode)
