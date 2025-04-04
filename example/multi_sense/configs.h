#pragma once

//INIT
// #define INIT_CAM_RADIUS     0.0f   //초기 카메라 위치 : 중심으로부터 거리
#define INIT_CAM_RADIUS     50.0f   //초기 카메라 위치 : 중심으로부터 거리
#define INIT_CAM_VANGLE     10.0f   //초기 카메라 위치 : 수직각도 // degree
#define INIT_CAM_HANGLE     50.0f   //초기 카메라 위치 : 수평각도 // degree

//window
#define WINDOW_WIDTH    1600        //창 너비
#define WINDOW_HEIGHT   900        //창 높이

// Constants
#define ORBIT_SPEED         0.01f   // 궤도 회전 속도
#define TRANSLATION_SPEED   0.5f    // 이동 속도
#define ZOOM_SPEED          1.0f    // 줌 속도
#define LINE_THICKNESS      2.0f    // 선 두께
#define CAMERA_SPEED        0.1f

// Scene set
#define GRID_NUM        16
#define GRID_Z_OFFSET   -0.01f
#define GRID_COEFFI     2

// video configure
#define VIDEO_SPEED     1                       //배속
#define VIDEO_SPEED_CONTROL_RESOLUTION  0.05f   //조절배속
#define VIDEO_SPEED_COEFFI_MAX  30.0f            //최대계수
#define VIDEO_SPEED_COEFFI_MIN  0.005f            //최소계수

// image sensor configure (camera)
#define LENS_HFOV       40  //degree
#define LENS_VFOV       40 //degree
#define LENS_ZOOM       3
