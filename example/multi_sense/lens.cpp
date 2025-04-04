
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "lens.h"
#include "vis_util.h"
#include "convert_coord.h"

std::vector<float> LensView::transposeMatrix(const std::vector<float>& matrix, int rows, int cols) {
    std::vector<float> transposed(cols * rows);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            transposed[j * rows + i] = matrix[i * cols + j]; // (i, j) -> (j, i)
        }
    }
    return transposed;
}

void LensView::vec2Series(){
    for(int i=0; i<lens_view_vector.size(); i++){
        lens_series.push_back(lens_view_vector[i].x);
        lens_series.push_back(lens_view_vector[i].y);
        lens_series.push_back(lens_view_vector[i].z);
    }
}

void LensView::pushBox_xywh(float x, float y, float w, float h){

}

void LensView::pushBox_xyxy(float x_start, float y_start, float x_end, float y_end){
// camera way : x
    std::vector<glm::vec3> box_region;

    glm::vec3 x_start_vec(0.0f, w.y * (x_start - screen_half_w) / screen_half_w, 0.0f);
    glm::vec3 y_start_vec(0.0f, 0.0f, h.z * (y_start - screen_half_h) / screen_half_h);
    glm::vec3 x_end_vec(0.0f, w.y * (x_end - screen_half_w) / screen_half_w, 0.0f);
    glm::vec3 y_end_vec(0.0f, 0.0f, h.z * (y_end - screen_half_h) / screen_half_h);

    glm::vec3 box_vec0 =  front - x_start_vec - y_start_vec;
    glm::vec3 box_vec1 =  front - x_end_vec - y_start_vec;
    glm::vec3 box_vec2 =  front - x_end_vec - y_end_vec;
    glm::vec3 box_vec3 =  front - x_start_vec - y_end_vec;

    box_region.push_back(box_vec0);
    box_region.push_back(box_vec1);
    box_region.push_back(box_vec2);
    box_region.push_back(box_vec3);

    box_regions.push_back(box_region);
}

void LensView::clearBox(){
    box_regions.clear();
    box_converted_regions.clear();
}

void LensView::convertBoxes(ConvertMatrix CM){

    std::vector<float> B;
    for(auto box : box_regions){
        for(int i=0; i<box.size(); i++){
            B.push_back(box[i].x);
            B.push_back(box[i].y);
            B.push_back(box[i].z);
            B.push_back(1.0f);
        }
    }

    auto B_T = transposeMatrix(B, 4, box_regions.size()*4);
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> matA(CM.convert_matrix.data());
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matB = 
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(B_T.data(), 4, box_regions.size()*4);
    Eigen::MatrixXf matC;

    matC.resize(box_regions.size()*4, 4);
    matC = matA  * matB;

    for(int i=0; i<box_regions.size(); i++){
        std::vector<glm::vec3> regions;
        regions.push_back(glm::vec3(matC(0, 4*i), matC(1, 4*i), matC(2, 4*i)));
        regions.push_back(glm::vec3(matC(0, 4*i+1), matC(1, 4*i+1), matC(2, 4*i+1)));
        regions.push_back(glm::vec3(matC(0, 4*i+2), matC(1, 4*i+2), matC(2, 4*i+2)));
        regions.push_back(glm::vec3(matC(0, 4*i+3), matC(1, 4*i+3), matC(2, 4*i+3)));

        box_converted_regions.push_back(regions);
    }
}

void LensView::drawBoxes(Space& space, glm::vec3 V, glm::vec3 color){
    for(auto box : box_converted_regions){
        space.addLine(V, box[0], color);
        space.addLine(V, box[1], color);
        space.addLine(V, box[2], color);
        space.addLine(V, box[3], color);
    }
}

void LensView::setScreenWH(float w, float h){
    this->screen_w = w;
    this->screen_h = h;
    this->screen_half_w = w/2.0f;
    this->screen_half_h = h/2.0f;
}

// std::vector<int> LensView::getInnerBoxPointIndex(Space& space, int box_index){
    
//     std::vector<int> ret;

//     std::vector<glm::vec3> _box = box_converted_regions[box_index];
//     Eigen::Vector3f V0(V.x, V.y, V.z);
//     Eigen::Vector3f P0(_box[0].x, _box[0].y, _box[0].z);
//     Eigen::Vector3f P1(_box[1].x, _box[1].y, _box[1].z);
//     Eigen::Vector3f P2(_box[2].x, _box[2].y, _box[2].z);
//     Eigen::Vector3f P3(_box[3].x, _box[3].y, _box[3].z);

//     std::vector<Eigen::Vector3f> normals;

//     normals.push_back(computeNormal(V0, P0, P1));
//     normals.push_back(computeNormal(V0, P1, P2));
//     normals.push_back(computeNormal(V0, P2, P3));
//     normals.push_back(computeNormal(V0, P3, P0));    

//     for(int i=0; i<space.points.size(); i+=2){
//     // for(auto point : space.points){
//         int _flag = 1;
//         int index = 0;
//         for(auto normal : normals){
//             float sign = glmToEigen(space.points[i].point - eigenToGlm(V0)).dot(normal);
//             if(sign < 0) {
//                 _flag = 0;
//                 break;
//             }
//             index++;
//         }
        
//         if(_flag){
//             ret.push_back(i);
//         }
//     }
//     return ret;
// }

std::vector<int> LensView::getInnerBoxPointIndex(Space& space, int box_index){
    std::vector<int> ret;

    const auto& _box = box_converted_regions[box_index];
    Eigen::Vector3f V0(V.x, V.y, V.z);
    Eigen::Vector3f P0(_box[0].x, _box[0].y, _box[0].z);
    Eigen::Vector3f P1(_box[1].x, _box[1].y, _box[1].z);
    Eigen::Vector3f P2(_box[2].x, _box[2].y, _box[2].z);
    Eigen::Vector3f P3(_box[3].x, _box[3].y, _box[3].z);

    std::vector<Eigen::Vector3f> normals;
    normals.reserve(4);  // 메모리 할당 최적화

    normals.push_back(computeNormal(V0, P0, P1));
    normals.push_back(computeNormal(V0, P1, P2));
    normals.push_back(computeNormal(V0, P2, P3));
    normals.push_back(computeNormal(V0, P3, P0));

    for (int i = 0; i < space.points.size(); i+=2) {
        Eigen::Vector3f D = glmToEigen(space.points[i].point) - V0;

        bool inside = true;
        for (const auto& normal : normals) {
            if (D.dot(normal) < 0) {
                inside = false;
                break;
            }
        }

        if (inside) ret.push_back(i);//space.points[i].color = glm::vec3(255.0f, 0.0f, 0.0f);
    }

    return ret;
}

Eigen::Vector3f glmToEigen(const glm::vec3& v) {
    return Eigen::Vector3f(v.x, v.y, v.z);
}

glm::vec3 eigenToGlm(const Eigen::Vector3f& v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

Eigen::Vector3f computeNormal(const Eigen::Vector3f& a,
                              const Eigen::Vector3f& b,
                              const Eigen::Vector3f& c) {
    return (b - a).cross(c - a).normalized();
}