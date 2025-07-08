#ifndef LENS_H
#define LENS_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "vis_util.h"
#include "convert_coord.h"

#define CPU_CONVERSION  0
#define GPU_CONVERSION  1

class LensView{
public:
    LensView();
    LensView(glm::vec3 front, glm::vec3 h, glm::vec3 w, glm::vec3 V){
        this->front = front;
        this->h = h;
        this->w = w;
        this->V = V;

        // must be kept this order
        glm::vec3 viewvec0 = front + h + w;
        glm::vec3 viewvec1 = front + h - w;
        glm::vec3 viewvec2 = front - h - w;
        glm::vec3 viewvec3 = front - h + w;

        this->lens_view_vector.push_back(viewvec0);
        this->lens_view_vector.push_back(viewvec1);
        this->lens_view_vector.push_back(viewvec2);
        this->lens_view_vector.push_back(viewvec3);

        vec2Series();
    };

    void vec2Series();
    void pushBox_xywh(float x, float y, float w, float h);
    void pushBox_xyxy(float x_start, float y_start, float x_end, float y_end);
    void clearBox();
    void setScreenWH(float w, float h);
    void convertBoxes(ConvertMatrix CM);
    void drawBoxes(Space& space, glm::vec3 V, glm::vec3 color);
    std::vector<float> transposeMatrix(const std::vector<float>& matrix, int rows, int cols);
    std::vector<int> getInnerBoxPointIndex(Space& space, int box_index);


// private:
    glm::vec3 front;
    glm::vec3 h;
    glm::vec3 w;
    glm::vec3 V;

    float screen_w;
    float screen_h;
    float screen_half_w;
    float screen_half_h;

    std::vector<glm::vec3> lens_view_vector;
    std::vector<float> lens_series;
    std::vector<std::vector<glm::vec3>> box_regions;
    std::vector<std::vector<glm::vec3>> box_converted_regions;
};
Eigen::Vector3f glmToEigen(const glm::vec3& v);
glm::vec3 eigenToGlm(const Eigen::Vector3f& v);
Eigen::Vector3f computeNormal(const Eigen::Vector3f& a,
                              const Eigen::Vector3f& b,
                              const Eigen::Vector3f& c);
#endif