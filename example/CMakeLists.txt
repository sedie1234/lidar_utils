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

# just client example
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

# visualize example
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

# record example
set(RECORD_SRC_FILES
    src/record_example.cpp)
add_executable(lidar_record_example ${RECORD_SRC_FILES})
target_link_libraries(lidar_record_example
    OusterSDK::ouster_client
    lidar_util
)
target_include_directories(lidar_record_example PUBLIC
    ${LIDAR_UTIL_DIR}
)

#load csv example
set(LOAD_SRC_FILES
    src/load_example.cpp
    src/camera.cpp)
add_executable(lidar_load_example ${LOAD_SRC_FILES})
target_link_libraries(lidar_load_example
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::GLEW
    glfw
    OusterSDK::ouster_client
    lidar_util
)
target_include_directories(lidar_load_example PUBLIC
    ${LIDAR_UTIL_DIR}
)

#multi sensor example
set(MULTISENSE_SRC_FILES
    multi_sense/multi_sense_example.cpp
    multi_sense/camera.cpp
    multi_sense/lens.cpp
	multi_sense/matmul.cu
)
add_executable(multi_sense_example ${MULTISENSE_SRC_FILES})
target_link_libraries(multi_sense_example
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::GLEW
    glfw
    OusterSDK::ouster_client
    lidar_util
    Eigen3::Eigen
    ${TORCH_LIBRARIES}
)
target_include_directories(multi_sense_example PUBLIC
    ${LIDAR_UTIL_DIR}
)

