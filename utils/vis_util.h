#ifndef VIS_UTIL_H
#define VIS_UTIL_H

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <cmath>
#include <tuple>

#include "lidar_util.h"

typedef struct _line{
    glm::vec3 start;
    glm::vec3 end;
    glm::vec3 color;
}Line;

typedef struct _point{
    glm::vec3 point;
    glm::vec3 color;
}Point;

typedef struct _box{
    glm::vec3 point;
    glm::vec3 xway;
    glm::vec3 yway;
    glm::vec3 zway;
    glm::vec3 color;
}Box;

class Space {
public:
    // utils for point
    void addPoint(const glm::vec3& point);
    void addPoint(const glm::vec3& point, const glm::vec3& color);
    void clearPoints();
    
    // utils for line
    void addLine(const glm::vec3& start, const glm::vec3& end,
                        const glm::vec3& color);
    void clearLines();

    // utils for box
    void addBox(const glm::vec3& point, const glm::vec3& xway, 
                const glm::vec3& yway, const glm::vec3& zway, const glm::vec3& color);
    void clearBoxes();

    // void addObj(const glm::vec3& point, const glm::vec3& xway, 
    //             const glm::vec3& yway, const glm::vec3& zway);
    void drawGrid(int grid_num, int grid_coeffi, float orbitRadius, float grid_z_offset);

    void lidarIntoSpace(std::vector<std::vector<LidarData>> lidar_data, int lidar_index);

    void render() const;

// private:
    //time series points
    std::vector<std::pair<int, std::vector<Point>>> ts_points; 
    std::vector<Point> points; //point, color
    //start point, end point, R, G, B
    // std::vector<std::tuple<glm::vec3, glm::vec3, GLfloat, GLfloat, GLfloat>> lines; 
    std::vector<Line> lines;
    std::vector<Box> boxes;
};


#endif  // VIS_UTIL_H