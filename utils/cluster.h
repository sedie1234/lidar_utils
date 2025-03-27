#ifndef CLUSTER_H
#define CLUSTER_H

#include <iostream>
#include "lidar_util.h"

enum ClusterStatus{
    NONE = -2,
    NOISE,
};

class DBScan{

public:
    DBScan(){

    };
    DBScan(std::vector<LidarData> lidar_data, float epsilon, int elems);

    bool clustering_fixindex(LidarData& a, int cluster_index);
    std::vector<int> clustering(LidarData a);
    int clustering2(std::vector<int>& indices, LidarData a);
    float distance(LidarData a, LidarData b);
    void run();
    void run2();

//private:
    std::vector<LidarData> lidar_data;
    float epsilon;
    int elems;
};

#endif