#pragma once

#include <cmath>  // for sin, cos, M_PI
#include <iostream>
#include <tuple>

#include "vis_util.h"

void Space::addPoint(const glm::vec3& point) {
    Point _point;
    _point.point = point;
    _point.color = glm::vec3(1.0f, 0.0f, 0.0f); //default = red
    points.push_back(_point);
}

void Space::addPoint(const glm::vec3& point, const glm::vec3& color){
    Point _point;
    _point.point = point;
    _point.color = color;

    // if(point.x > 0 && point.y < 0)
        points.push_back(_point);
}

void Space::clearPoints(){
    points.clear();
    std::vector <Point>().swap(points);
}

void Space::addLine(const glm::vec3& start, const glm::vec3& end,
                    const glm::vec3& color) {
    Line _line;
    _line.start = start;
    _line.end = end;
    _line.color = color;
    lines.push_back(_line);
}

void Space::clearLines(){
    lines.clear();
    std::vector <Line>().swap(lines);
}

void Space::addBox(const glm::vec3& point, const glm::vec3& xway, 
                const glm::vec3& yway, const glm::vec3& zway, const glm::vec3& color){
    Box _box;
    _box.point = point;
    _box.xway = xway;
    _box.yway = yway;
    _box.zway = zway;
    _box.color = color;
    boxes.push_back(_box);
}

void Space::clearBoxes(){
    boxes.clear();
    std::vector <Box>().swap(boxes);
}

// void Space::drawGrid(){
//     // Draw grid
//     glBegin(GL_LINES);
//     glColor3f(0.5f, 0.5f, 0.5f);  // 회색
//     for (int i = -10; i <= 10; ++i) {
//         glVertex3f(-10.0f, 0.0f, i);
//         glVertex3f(10.0f, 0.0f, i);
//         glVertex3f(i, 0.0f, -10.0f);
//         glVertex3f(i, 0.0f, 10.0f);
//     }
//     glEnd();
// }

void Space::drawGrid(int grid_num, int grid_coeffi, float orbitRadius, float grid_z_offset){
    std::vector<std::pair<glm::vec3, glm::vec3>> gridpoints;

    for(int i=-grid_num; i<grid_num+1; i++){
        std::pair<glm::vec3, glm::vec3> line0_points;
        line0_points.first = glm::vec3(grid_coeffi*orbitRadius*i/grid_num, -grid_coeffi*orbitRadius, grid_z_offset);
        line0_points.second = glm::vec3(grid_coeffi*orbitRadius*i/grid_num, grid_coeffi*orbitRadius, grid_z_offset);
        gridpoints.push_back(line0_points);

        std::pair<glm::vec3, glm::vec3> line1_points;
        line1_points.first = glm::vec3(-grid_coeffi*orbitRadius, grid_coeffi*orbitRadius*i/grid_num, grid_z_offset);
        line1_points.second = glm::vec3(grid_coeffi*orbitRadius, grid_coeffi*orbitRadius*i/grid_num, grid_z_offset);
        gridpoints.push_back(line1_points);
    }

    for(const auto &points : gridpoints){
        const glm::vec3 color(0.6f, 0.6f, 0.6f); // gray color
        addLine(points.first, points.second, color); 
    }
}

void Space::render() const{
    // Draw points
    for (const auto& point : points) {
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        glColor3f(point.color.x, point.color.y, point.color.z);
        glVertex3f(point.point.x, point.point.y, point.point.z);
        glEnd();
    }

    // Draw lines
    for (const auto& line : lines) {
        glBegin(GL_LINES);
        glColor3f(line.color.x, line.color.y, line.color.z);
        glVertex3f(line.start.x, line.start.y, line.start.z);
        glVertex3f(line.end.x, line.end.y, line.end.z);
        glEnd();
    }

    // Draw boxes
    for (const auto& box : boxes) {
        
        glBegin(GL_LINES);
        glColor3f(box.color.x, box.color.y, box.color.z);

        float x = box.point.x;
        float y = box.point.y;
        float z = box.point.z;

        glVertex3f(x, y, z);        glVertex3f(x + box.xway.x, y + box.xway.y, z + box.xway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.yway.x, y + box.yway.y, z + box.yway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.zway.x, y + box.zway.y, z + box.zway.z);
        
        x = box.point.x + box.xway.x + box.yway.x;
        y = box.point.y + box.xway.y + box.yway.y;
        z = box.point.z + box.xway.z + box.yway.z;

        glVertex3f(x, y, z);        glVertex3f(x - box.xway.x, y - box.xway.y, z - box.xway.z);
        glVertex3f(x, y, z);        glVertex3f(x - box.yway.x, y - box.yway.y, z - box.yway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.zway.x, y + box.zway.y, z + box.zway.z);

        x = box.point.x + box.yway.x + box.zway.x;
        y = box.point.y + box.yway.y + box.zway.y;
        z = box.point.z + box.yway.z + box.zway.z;

        glVertex3f(x, y, z);        glVertex3f(x - box.yway.x, y - box.yway.y, z - box.yway.z);
        glVertex3f(x, y, z);        glVertex3f(x - box.zway.x, y - box.zway.y, z - box.zway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.xway.x, y + box.xway.y, z + box.xway.z);

        x = box.point.x + box.zway.x + box.xway.x;
        y = box.point.y + box.zway.y + box.xway.y;
        z = box.point.z + box.zway.z + box.xway.z;

        glVertex3f(x, y, z);        glVertex3f(x - box.zway.x, y - box.zway.y, z - box.zway.z);
        glVertex3f(x, y, z);        glVertex3f(x - box.xway.x, y - box.xway.y, z - box.xway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.yway.x, y + box.yway.y, z + box.yway.z);

        glEnd();
    }
}

void Space::lidarIntoSpace(std::vector<std::vector<LidarData>> lidar_data, int lidar_index){
    for(int i=0; i<lidar_data[lidar_index].size(); i++){
        glm::vec3 point(lidar_data[lidar_index][i].x, lidar_data[lidar_index][i].y, lidar_data[lidar_index][i].z);
        // glm::vec3 color(lidar_data[lidar_index][i].reflectivity, 0.0f, 0.0f);
        float color_sense = lidar_data[lidar_index][i].reflectivity / 26.54f;
        glm::vec3 color(0.0f, 0.973f - color_sense, 0.364f + color_sense);
        addPoint(point, color);
    }
}