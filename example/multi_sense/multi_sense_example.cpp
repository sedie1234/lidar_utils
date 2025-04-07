#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>

#include "camera.h"
#include "lidar_util.h"
#include "configs.h"
#include "vis_util.h"
#include "uart_util.h"
#include "convert_coord.h"
#include "infer_util.h"
#include "lens.h"
#include "cluster.h"


#include "opencv2/opencv.hpp"

#define MAX_READ_SIZE   235
#define CMD_DELAY   100000

bool leftMousePressed = false;
double lastMouseX = 0.0, lastMouseY = 0.0;
float horizontalAngle = glm::radians(INIT_CAM_HANGLE * 360.0f / 100.0f), verticalAngle = glm::radians(INIT_CAM_VANGLE * 360.0f / 100.0f); // 카메라의 회전 각도
float orbitRadius = INIT_CAM_RADIUS;                           // 궤도 반지름

int video_control = 0;
int video_mode = 0;

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

// imu, uart functions
void send_command(UartDevice* dev, char* cmd);
void read_response(UartDevice* dev, char* read_data);
std::vector<float> transposeMatrix(const std::vector<float>& matrix, int rows, int cols);
bool distanceRule(Coordinate t, LidarData a, LidarData b);
float distanceAtoB(Coordinate t, LidarData a);

int main(int argc, char* argv[]) {

    std::vector<std::string> hostname;

    if(argc<5){
        std::cout << "Usage : ./vis_example [imu0 to 1 x] [imu0 to 1 y] [imu0 to 1 z] [hostname]" << std::endl;
        return -1;
    }

    // argument set for conversion matrix 
    std::string __x = argv[1];
    std::string __y = argv[2];
    std::string __z = argv[3];

    ConvertMatrix CM;

    Coordinate t;

    t.x = stof(__x);
    t.y = stof(__y);
    t.z = stof(__z);

    CM.setTransition(t);

    // for image sensor view vector
    //x
    glm::vec3 lens_way(LENS_ZOOM*1.0f, 0.0f, 0.0f);
    glm::vec3 lens_h(0.0f, 0.0f, LENS_ZOOM*glm::tan(glm::radians(LENS_VFOV/2.0f)));
    glm::vec3 lens_w(0.0f, LENS_ZOOM*glm::tan(glm::radians(LENS_HFOV/2.0f)), 0.0f);

    //y
    // glm::vec3 lens_way(0.0f, 1.0f, 0.0f);
    // glm::vec3 lens_h(0.0f, 0.0f, glm::sin(glm::radians(LENS_VFOV/2.0f)));
    // glm::vec3 lens_w(glm::sin(glm::radians(LENS_HFOV/2.0f)), 0.0f, 0.0f);

    //z
    // glm::vec3 lens_way(0.0f, 0.0f, 1.0f);
    // glm::vec3 lens_h(0.0f, glm::sin(glm::radians(LENS_VFOV/2.0f)), 0.0f);
    // glm::vec3 lens_w(glm::sin(glm::radians(LENS_HFOV/2.0f)), 0.0f, 0.0f);

    LensView lens(lens_way, lens_h, lens_w, glm::vec3(t.x, t.y, t.z));
    std::vector<float> lens_viewvector;

    for(int i=0; i<lens.lens_view_vector.size(); i++){
        lens_viewvector.push_back(lens.lens_view_vector[i].x);
        lens_viewvector.push_back(lens.lens_view_vector[i].y);
        lens_viewvector.push_back(lens.lens_view_vector[i].z);
        lens_viewvector.push_back(1.0f);
    }

    // get hostnames
    for(int i=4; i<argc; i++){
        hostname.push_back(argv[i]);
    }

    // lidar init
    int lidar_index = 0;
    LidarUtil _lidar;
    for(auto &host: hostname){
        _lidar.pushHost(host); // register lidar
    }
    
    ouster::sensor::SensorScanSource source(_lidar.sensors); // lidar start : make thread in function
    _lidar.getSensorInfo(source);
    _lidar.printInfo(lidar_index); // print lidar info : not necessary

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

    /**** uart ****/
    struct UartDevice imu0 = {
        .filename = "/dev/ttyUSB0",
        .rate = B115200
    };

    struct UartDevice imu1 = {
        .filename = "/dev/ttyUSB1",
        .rate = B115200
    };

    int rc = uart_start(&imu0, false);
    if (rc) {
        printf("Failed to start IMU0\n");
        return rc;
    }

    rc = uart_start(&imu1, false);
    if (rc) {
        printf("Failed to start IMU1\n");
        return rc;
    }

    send_command(&imu0, "sp=10");
    send_command(&imu0, "ss=16");

    send_command(&imu1, "sp=10");
    send_command(&imu1, "ss=16");

    /****  inference ****/
    PTModel _model("../../yolov5n.torchscript.pt");
    cv::VideoCapture cap(0, cv::CAP_V4L2);

    if(!cap.isOpened()){
        std::cerr << "Fail to open camera\n";
        return -1;
    }

    

    cv::Mat frame;
    /**** Main loop ****/ 
    while (!glfwWindowShouldClose(window)) {

        // get IMU data
        char imu0_str[MAX_READ_SIZE];
        char imu1_str[MAX_READ_SIZE];
        read_response(&imu0, imu0_str);
        read_response(&imu1, imu1_str);

        // Quaternion imu0_data = parse_imu(imu0_str);
        // Quaternion imu1_data = parse_imu(imu1_str);

        Quaternion imu0_data, imu1_data;

        sscanf(imu0_str, "%f %f %f %f", &imu0_data.w, &imu0_data.x, &imu0_data.y, &imu0_data.z);
        sscanf(imu1_str, "%f %f %f %f", &imu1_data.w, &imu1_data.x, &imu1_data.y, &imu1_data.z);

        // get convert matrix
        CM.setStartQuaternion(imu0_data);
        CM.setDstQuaternion(imu1_data);
        CM.setStartToDstQuaternion();

        CM.makeConvertMatrix();

        // set lens view vector
        auto B_T = transposeMatrix(lens_viewvector, 4, 4);

        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> matA(CM.convert_matrix.data());
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matB = 
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(B_T.data(), 4, 4);
        Eigen::MatrixXf matC;

        matC.resize(4, 4);
        matC = matA * matB;

        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // draw scene
        space.clearLines();
        space.clearBoxes();
        
        space.drawGrid(GRID_NUM, GRID_COEFFI, orbitRadius, GRID_Z_OFFSET);

        // add lens lines
        glm::vec3 color_yellow(255.0f, 255.0f, 0.0f);
        glm::vec3 color_red(255.0f, 0.0f, 0.0f);
        glm::vec3 color_purple(255.0f, 0.0f, 255.0f);
        space.addLine(glm::vec3(t.x, t.y, t.z), glm::vec3(matC(0,0), matC(1,0), matC(2,0)), color_yellow);
        space.addLine(glm::vec3(t.x, t.y, t.z), glm::vec3(matC(0,1), matC(1,1), matC(2,1)), color_yellow);
        space.addLine(glm::vec3(t.x, t.y, t.z), glm::vec3(matC(0,2), matC(1,2), matC(2,2)), color_yellow);
        space.addLine(glm::vec3(t.x, t.y, t.z), glm::vec3(matC(0,3), matC(1,3), matC(2,3)), color_yellow);

        // get lidar data & draw lidar data
        if(_lidar.wholeScan(source)){
            space.clearPoints();       
            if (video_control == 0){
                for(int i=0; i<_lidar.lidar_data.size(); i++){
                    space.lidarIntoSpace(_lidar.lidar_data, i);
                }
            }
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

        //mode 1 : only lens view
        if(video_mode == 1){

            std::vector<Point> temp_points = space.points;
            space.points.clear();

            //get normalizes
            Eigen::Vector3f V(t.x, t.y, t.z);
            Eigen::Vector3f P0(matC(0,0), matC(1,0), matC(2,0));
            Eigen::Vector3f P1(matC(0,1), matC(1,1), matC(2,1));
            Eigen::Vector3f P2(matC(0,2), matC(1,2), matC(2,2));
            Eigen::Vector3f P3(matC(0,3), matC(1,3), matC(2,3));

            std::vector<Eigen::Vector3f> normals;

            normals.push_back(computeNormal(V, P0, P1));
            normals.push_back(computeNormal(V, P1, P2));
            normals.push_back(computeNormal(V, P2, P3));
            normals.push_back(computeNormal(V, P3, P0));
            
            for(auto point : temp_points){
                int _flag = 1;
                int index = 0;
                for(auto normal : normals){
                    float sign = glmToEigen(point.point - eigenToGlm(V)).dot(normal);
                    if(sign < 0) {
                        _flag = 0;
                        break;
                    }
                    index++;
                }
                
                if(_flag){
                    space.points.push_back(point);
                    
                }
            }
            // std::cout << space.points.size() << std::endl;
        }

        //inference mode
        if(video_mode > 1){
            cap >> frame;
            if (frame.empty()) std::cout << "frame is empty!\n";

            _model.pushInput(_model.matToTensor(frame));
            _model.run();
            _model.predProcess();
            _model.drawPredBoxes(frame);

            cv::imshow("inference", frame);
            cv::waitKey(1);

            if(_model.rects.size()){

                lens.clearBox();
                lens.setScreenWH(frame.rows, frame.cols);
                for(auto rect : _model.rects){
                    rect.x *= frame.rows / 640.0f;
                    rect.y *= frame.cols / 640.0f;
                    rect.width *= frame.rows / 640.0f;
                    rect.height *= frame.cols / 640.0f;
                    lens.pushBox_xyxy(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
                    break; // nms 귀찮음...
                }
                lens.convertBoxes(CM);
                lens.drawBoxes(space, glm::vec3(t.x, t.y, t.z), color_red);

                if(video_mode > 2){
                    std::vector<int> inner_box_points = lens.getInnerBoxPointIndex(space, 0);

                    if(video_mode == 3){
                        for(auto pidx : inner_box_points){
                            space.pointColorChange(pidx, color_red);
                        }
                    }
                    if(video_mode == 4){
                        std::vector<LidarData> lens_lidar;
                        if(inner_box_points.size() <2) continue;
                        for(auto pidx : inner_box_points){
                            LidarData temp_ldata;
                            temp_ldata.x = space.points[pidx].point.x+0.01f;
                            temp_ldata.y = space.points[pidx].point.y;
                            temp_ldata.z = space.points[pidx].point.z;
                            lens_lidar.push_back(temp_ldata);
                        }
                        std::sort(lens_lidar.begin(), lens_lidar.end(), [&](LidarData a, LidarData b) {
                            return distanceRule(t, a, b);
                        });
                        std::cout << "distance camere to object front side : " << distanceAtoB(t, lens_lidar[0]) << "m\n";
                        DBScan dbs(lens_lidar, 1.0f, 10);
                        std::vector<int> cluster_list;
                        int cluster_index =0;
                        while(cluster_list.size()<3){
                            cluster_list.clear();
                            dbs.clustering2(cluster_list, lens_lidar[cluster_index]);
                            cluster_index++;
                            if(cluster_index > 3) break;
                        }

                        if(cluster_list.size()){
                            glm::vec3 minp(9999.0, 9999.0, 9999.0), maxp(-9999.0, -9999.0, -9999.0);
                            
                            for(auto cidx : cluster_list){

                                space.addPoint(glm::vec3(lens_lidar[cidx].x, 
                                                        lens_lidar[cidx].y,
                                                        lens_lidar[cidx].z), color_red);
                                if(minp.x > lens_lidar[cidx].x) minp.x = lens_lidar[cidx].x;
                                if(minp.y > lens_lidar[cidx].y) minp.y = lens_lidar[cidx].y;
                                if(minp.z > lens_lidar[cidx].z) minp.z = lens_lidar[cidx].z;
                                if(maxp.x < lens_lidar[cidx].x) maxp.x = lens_lidar[cidx].x;
                                if(maxp.y < lens_lidar[cidx].y) maxp.y = lens_lidar[cidx].y;
                                if(maxp.z < lens_lidar[cidx].z) maxp.z = lens_lidar[cidx].z;
                            }

                            space.addBox(minp, maxp, color_red);
                            // space.addBox(glm::vec3(lens_lidar[cluster_list[0]].x, lens_lidar[cluster_list[0]].y, lens_lidar[cluster_list[0]].z),
                            //             glm::vec3(lens_lidar[cluster_list[cluster_list.size()-1]].x, 
                            //                     lens_lidar[cluster_list[cluster_list.size()-1]].y,
                            //                     lens_lidar[cluster_list[cluster_list.size()-1]].z), 
                            //             color_red);
                        }
                        
                        
                    }
                }
            }

            _model.clearInputs();
        }

        // render scene
        space.render();

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    // uart terminate
    uart_stop(&imu0);
    uart_stop(&imu1);

    glfwDestroyWindow(window);
    glfwTerminate();
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
        } else if (key == GLFW_KEY_ESCAPE) {                //esc
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_P) {                     //p
            if(video_control != 1)
                video_control = 1;                          //pause
            else
                video_control = 0;                          //play
        // visualize mode // 0 : normal 
                          // 1 : partial view 
                          // 2 : camera view 
                          // 3 : inference view 1 
                          // 4 : inference view 2
        } else if (key == GLFW_KEY_M) {                     
            video_mode++;
            video_mode = video_mode % 5;
            std::cout << "video mode : " << video_mode << std::endl;
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

void send_command(UartDevice* dev, char* cmd) {
    printf("Sending: %s\n", cmd);
    uart_writes(dev, cmd);
    uart_writes(dev, "\r\n");
    usleep(CMD_DELAY);
}

void read_response(UartDevice* dev, char* read_data) {
    // char read_data[MAX_READ_SIZE];
    int len = uart_reads(dev, read_data, MAX_READ_SIZE);
}

std::vector<float> transposeMatrix(const std::vector<float>& matrix, int rows, int cols) {
    std::vector<float> transposed(cols * rows);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            transposed[j * rows + i] = matrix[i * cols + j]; // (i, j) -> (j, i)
        }
    }
    return transposed;
}

bool distanceRule(Coordinate t, LidarData a, LidarData b){
    float ax = t.x - a.x;
    float ay = t.y - a.y;
    float az = t.z - a.z;

    float bx = t.x - b.x;
    float by = t.y - b.y;
    float bz = t.z - b.z;
    
    float distance_a = ax*ax + ay*ay + az*az;
    float distance_b = bx*bx + by*by + bz*bz;

    return distance_a < distance_b;
}

float distanceAtoB(Coordinate t, LidarData a){
    float ax = t.x - a.x;
    float ay = t.y - a.y;
    float az = t.z - a.z;
    
    float distance_a = ax*ax + ay*ay + az*az;

    return sqrt(distance_a);
}