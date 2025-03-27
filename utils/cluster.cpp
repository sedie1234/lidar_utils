#include <iostream>
#include <vector>
#include <algorithm>

#include "cluster.h"
#include "lidar_util.h"

DBScan::DBScan(std::vector<LidarData> lidar_data, float epsilon, int elems){

    // std::copy(lidar_data.begin(), lidar_data.end(), this->lidar_data.begin());
    // this->lidar_data.assign(lidar_data.begin(), lidar_data.end());
    this->lidar_data = lidar_data;

    for(auto& dat : this->lidar_data){
        dat.cluster_id = NONE;
    }

    this->epsilon = epsilon;
    this->elems = elems;
}

float DBScan::distance(LidarData a, LidarData b){
    float x = b.x - a.x;
    float y = b.y - a.y;
    float z = b.z - a.z;

    return x*x + y*y + z*z;
}

bool DBScan::clustering_fixindex(LidarData& a, int cluster_index){
    std::vector<int> indices;
    for(int i=0; i<lidar_data.size(); i++){
        if(distance(a, lidar_data[i]) < epsilon * epsilon)
            indices.push_back(i);
    }

    if(indices.size() > elems){
        for(auto i : indices){
            a.cluster_id = cluster_index;
            lidar_data[i].cluster_id = cluster_index;
        }
        return true;
    }else{
        a.cluster_id = NOISE;
        return false;
    }
}

std::vector<int> DBScan::clustering(LidarData a){
    std::vector<int> ret;
    for(int i=0; i<lidar_data.size(); i++){
        if(distance(a, lidar_data[i]) < epsilon * epsilon){
            ret.push_back(i);
        }
    }

    return ret;
}

int DBScan::clustering2(std::vector<int>& indices, LidarData a){
    
    int max_dis = 0;
    int count =0;
    int max_elem;

    int ret = -1;

    std::sort(indices.begin(), indices.end());
    for(int i=0; i<lidar_data.size(); i++){
        if(lidar_data[i].cluster_id == NOISE) continue;

        float dis = distance(a, lidar_data[i]);

        if( dis < epsilon * epsilon){
            bool found = std::binary_search(indices.begin(), indices.end(), i);
            if(!found){
                if(dis > max_dis){
                    max_elem = i;
                    max_dis = dis;
                }
                indices.push_back(i);
            }
            count++;
        }

    }

    // std::cout << "count, max_dis : " <<  count  << ", " << max_dis << std::endl;

    if(count > elems-1 && max_dis > 0){
        clustering2(indices, lidar_data[max_elem]);
    }
}

void DBScan::run(){
    int cluster_index=0;
    for(int i=0; i<lidar_data.size(); i++){
        if(lidar_data[i].cluster_id == NONE){
            std::vector<int> dat_index;
            dat_index = clustering(lidar_data[i]);
            if(dat_index.size() < elems) lidar_data[i].cluster_id = NOISE;
            else{
                int put_idx=cluster_index;
                for(auto idx : dat_index){
                    if(lidar_data[idx].cluster_id !=NONE && lidar_data[idx].cluster_id !=NOISE){
                        put_idx = lidar_data[idx].cluster_id;
                        break;
                    }
                }
                for(auto idx  : dat_index){
                    lidar_data[idx].cluster_id = put_idx;
                }
                cluster_index++;
            }
        }
        
    }
}

void DBScan::run2(){
    int cluster_index=0;

    for(int i=0; i<lidar_data.size(); i++){
        if(lidar_data[i].cluster_id == NONE){
            std::vector<int> dat_index;
            clustering2(dat_index, lidar_data[i]);
            // std::cout  << "cluster size : " << dat_index.size() << std::endl;
            if(dat_index.size() < elems) lidar_data[i].cluster_id = NOISE;
            else{
                for(auto idx : dat_index){
                    lidar_data[idx].cluster_id = cluster_index;
                }
                cluster_index++;
            }
        }
    }

}