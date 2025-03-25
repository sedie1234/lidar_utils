
# OusterSDK
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/OusterSDK" ${CMAKE_PREFIX_PATH})
find_package(OusterSDK REQUIRED COMPONENTS Static)

# jsoncpp library
find_package(jsoncpp REQUIRED)
include_directories(/usr/include/jsoncpp)
link_directories(/usr/lib/x86_64-linux-gnu)

# eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_library(lidar_util STATIC
    lidar_util.cpp
    vis_util.cpp
)

target_link_libraries(lidar_util
    jsoncpp
    # OusterSDK::OusterSDK
    OusterSDK::ouster_client
    Eigen3::Eigen
)