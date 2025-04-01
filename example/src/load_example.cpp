
#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <iostream>
#include <string>

#include "camera.h"
#include "lidar_util.h"
#include "configs.h"
#include "vis_util.h"

#if CV_VIEW
#include "opencv2/opencv.hpp"
#endif

bool leftMousePressed = false;
double lastMouseX = 0.0, lastMouseY = 0.0;
float horizontalAngle = glm::radians(INIT_CAM_HANGLE * 360.0f / 100.0f), verticalAngle = glm::radians(INIT_CAM_VANGLE * 360.0f / 100.0f); // 카메라의 회전 각도
float orbitRadius = INIT_CAM_RADIUS;                           // 궤도 반지름

int video_control = 0;
int frame_index = 0;

// camera position offset
glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f);
glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f);
glm::vec3 up(0.0f, 1.0f, 0.0f);

float x = orbitRadius * glm::cos(verticalAngle) * glm::cos(horizontalAngle) + cameraPosition.x;
float y = orbitRadius * glm::cos(verticalAngle) * glm::sin(horizontalAngle) + cameraPosition.y;
float z = orbitRadius * glm::sin(verticalAngle) + cameraPosition.z;

// Function Prototypes
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
void setupViewport(const GLuint width, const GLuint height);

int main(int argc, char* argv[]) {

    std::string loadfile;
    if(argc<2){
        std::cout << "Usage : ./record_example [loadfile.csv]" << std::endl;
        return -1;
    }

    //get save filename
    loadfile = argv[1];

    std::vector<std::vector<std::string>> read_data;
    std::ifstream file_(loadfile);

    if(!file_.is_open()){
        return -1;
    }

    std::cout << "Loading data ... " << loadfile << std::endl;
    // csv file parse to _lidar
    std::string line;
    std::vector<std::vector<LidarData>> _lidar;
    std::vector<LidarData> ldata;
    LidarData _data;
    int index = 0;
    
    while(std::getline(file_, line)){
        
        std::stringstream ss(line);
        std::string cell;
        int offset = 0;
        int lidar_index =0;
        while(std::getline(ss, cell, ',')){
            // std::cout << offset << ", " << cell << std::endl;
            
            if(offset == 0){
                lidar_index = std::stoi(cell);
                offset++;
                // if(lidar_index != index){
                //     index =  lidar_index;
                // }
            }else if(offset == 1){
                _data.x = std::stof(cell);
                offset++;
            }else if(offset == 2){
                _data.y = std::stof(cell);
                offset++;
            }else if(offset == 3){
                _data.z = std::stof(cell);
                offset++;
            }else if(offset == 4){
                _data.reflectivity =  std::stoi(cell);
                ldata.push_back(_data);
                offset = 0;
            }
        }
        if(lidar_index != index){
            index =  lidar_index;
            _lidar.push_back(ldata);
            ldata.clear();
        }
        
    }


    /**** Initialize GLFW ****/ 
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Orbiting Camera", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    setupViewport(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    /**** Set callbacks ****/ 
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);

    /**** Set scene ****/
    Space space;
    Camera camera;
    int prev_frame_index = 0;    


    std::cout << _lidar.size() << std::endl;
    /**** Main loop ****/ 
    while (!glfwWindowShouldClose(window)) {
        

        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // draw scene
        
        space.clearLines();
        space.clearBoxes();
        
        space.drawGrid(GRID_NUM, GRID_COEFFI, orbitRadius, GRID_Z_OFFSET);

    
        if(prev_frame_index != frame_index){
            space.clearPoints();
            std::cout << frame_index << "th frame\n";
            prev_frame_index = frame_index;
            if(frame_index > _lidar.size()-1) frame_index=_lidar.size()-1;
            std::cout << "lidar data size : " << _lidar[frame_index].size() << std::endl;
            space.lidarIntoSpace(_lidar, frame_index);
        }

        // Update camera position based on spherical coordinates
        float x = orbitRadius * glm::cos(verticalAngle) * glm::cos(horizontalAngle) + cameraPosition.x;
        float y = orbitRadius * glm::cos(verticalAngle) * glm::sin(horizontalAngle) + cameraPosition.y;
        float z = orbitRadius * glm::sin(verticalAngle) + cameraPosition.z;
        camera.setPosition(glm::vec3(x, y, z));
        camera.setTarget(cameraTarget);

        // Apply camera view matrix
        glm::mat4 view = camera.getViewMatrix();
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(&view[0][0]);

        // render scene
        space.render();

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    glfwDestroyWindow(window);
    glfwTerminate();
    file_.close();
    return 0;

}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        glm::vec3 cameraDirection(glm::cos(horizontalAngle),
                                  glm::sin(horizontalAngle),
                                  0.0f);
        glm::vec3 cameraOrthoDirection(-glm::sin(horizontalAngle),
                                       glm::cos(horizontalAngle),
                                       0.0f);
        if (key == GLFW_KEY_W) {                            //w
            cameraPosition -= CAMERA_SPEED * cameraDirection; // 앞으로 이동
            cameraTarget -= CAMERA_SPEED * cameraDirection; // 앞으로 이동
        } else if (key == GLFW_KEY_S) {                     //s
            cameraPosition += CAMERA_SPEED * cameraDirection; // 뒤로 이동
            cameraTarget += CAMERA_SPEED * cameraDirection; // 뒤로 이동
        } else if (key == GLFW_KEY_A) {                     //a
            cameraPosition -= CAMERA_SPEED * cameraOrthoDirection; // 왼쪽 이동
            cameraTarget -= CAMERA_SPEED * cameraOrthoDirection; // 왼쪽 이동
        } else if (key == GLFW_KEY_D) {                     //d
            cameraPosition += CAMERA_SPEED * cameraOrthoDirection; // 오른쪽 이동
            cameraTarget += CAMERA_SPEED * cameraOrthoDirection; // 오른쪽 이동
        } else if (key == GLFW_KEY_Q) {                     //d
            if(frame_index > 0) frame_index --;
        } else if (key == GLFW_KEY_E) {                     //d
            frame_index ++;
        } else if (key == GLFW_KEY_ESCAPE) {                //esc
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_P) {                     //p
            if(video_control != 1)
                video_control = 1;                          //pause
            else
                video_control = 0;                          //play
        } 
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        leftMousePressed = (action == GLFW_PRESS);
    }
}


void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    if (leftMousePressed) {
        double dx = xpos - lastMouseX;
        double dy = ypos - lastMouseY;

        horizontalAngle += (float)(dx * ORBIT_SPEED);
        verticalAngle -= (float)(dy * ORBIT_SPEED);
        
        //when I click first_click, why vertialAngle go to zero???

        // 상하 회전 제한 (90도 범위 유지)
        if (verticalAngle > glm::radians(89.0f)) verticalAngle = glm::radians(89.0f);
        if (verticalAngle < glm::radians(-89.0f)) verticalAngle = glm::radians(-89.0f);


    }
    lastMouseX = xpos;
    lastMouseY = ypos;

}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    orbitRadius -= (float)(yoffset * ZOOM_SPEED);
    if (orbitRadius < 1.0f) orbitRadius = 1.0f; // 최소 거리 제한
}

void setupViewport(const GLuint width, const GLuint height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, static_cast<double>(width) / height, 0.1, 100.0);
}