## Requirements

1. install glut, glfw, glew, glm
```
$ sudo apt install freeglut3-dev libglu1-mesa-dev mesa-common-dev
$ sudo apt install mesa-utils
$ sudo apt install libglew-dev
$ sudo apt isntall libglfw3-dev libglfw3
$ sudo apt install libglm-dev
```

2. install third_party
 - requirements for ouster : eigen, jsoncpp, curl, spd, pcap, libtin, flatbuffers, png, optional-lite
```
$ sudo apt install libeigen3-dev
$ sudo apt install libjsoncpp-dev
$ sudo apt install libcurl4-openssl-dev
$ sudo apt install libspdlog-dev
$ sudo apt install libpcap-dev
$ sudo apt install libtins-dev
$ sudo apt install flatbuffers-compiler libflatbuffers-dev
$ sudo apt install libpng-dev
```
 - optional lite
```
$ cd third_party
$ git clone https://github.com/martinmoene/optional-lite.git
$ cd optional-lite
$ mkdir build && cd build
$ cmake ..
$ make
```
 - ouster sdk 
```
$ cd third_party
$ git clone https://github.com/ouster-lidar/ouster_example.git
$ cd ouster_example
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

## build
```
$ mkdir build && cd build
$ cmake .. -DCV_VIEW=1 -DPANORAMA=1 -DBUILD_VIS=ON
$ make
```

## cmake options
-DCV_VIEW : 1이면 opencv사용, 0이면 opencv 사용 안함
-DPANORAMA : 1이면 panorama view 생성, 0이면 생성 안함
-DBUILD_VIS : ON이면 visualizer example 생성, OFF면 생성 안함
-DCLUSTERING : 0이면 reflectivity, 1이면 clustering 알고리즘 1, 2면 clustering 알고리즘 2가 적용됨

## execute
```
$ cd build/example/
$ ./lidar_client_example [lidar addr]
$ ./lidar_vis_example [lidar addr]
```

