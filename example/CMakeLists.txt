find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)

# OpenGL, GLFW Library
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

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

set(CLIENT_SRC_FILES
    src/example.cpp
)
add_executable(lidar_client_example ${CLIENT_SRC_FILES})
target_link_libraries(lidar_client_example
    OusterSDK::ouster_client
    Eigen3::Eigen
    ${PCAP_LIBRARIES}
    lidar_util
)
target_include_directories(lidar_client_example PUBLIC
    ${LIDAR_UTIL_DIR}
)




if(BUILD_VIS)

set(VIS_SRC_FILES
    src/vis_example.cpp
    src/camera.cpp
)
add_executable(lidar_vis_example ${VIS_SRC_FILES})
target_link_libraries(lidar_vis_example
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::GLEW
    glfw
    jsoncpp
    OusterSDK::ouster_client
    Eigen3::Eigen
    ${PCAP_LIBRARIES}
    lidar_util
)
target_include_directories(lidar_vis_example PUBLIC
    ${LIDAR_UTIL_DIR}
)

endif()