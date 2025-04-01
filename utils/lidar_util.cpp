#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

#include "lidar_util.h"

void LidarUtil::pushHost(const std::string hostname, const std::string udp_dest, const int udp_port_lidar){
    ouster::sensor::sensor_config config;
    config.udp_dest = udp_dest;
    config.udp_port_lidar = udp_port_lidar;
    ouster::sensor::Sensor s(hostname, config);
    sensors.push_back(s);
    hostnames.push_back(hostname);
}

void LidarUtil::pushHost(const std::string hostname){
    ouster::sensor::sensor_config config;
    config.udp_dest = "@auto";
    ouster::sensor::Sensor s(hostname, config);
    sensors.push_back(s);
    hostnames.push_back(hostname);
}

void LidarUtil::pushHost(const std::string hostname, const ouster::sensor::sensor_config config){
    ouster::sensor::Sensor s(hostname, config);
    sensors.push_back(s);
}

ouster::sensor::SensorScanSource* LidarUtil::getSensorSource(){
    ouster::sensor::SensorScanSource s(sensors);
    return &s;
}

void LidarUtil::getSensorInfo(ouster::sensor::SensorScanSource& source){
    sensor_info = source.get_sensor_info();
    for (const auto& info : sensor_info) {
        luts.push_back(ouster::make_xyz_lut(
            info, true /* if extrinsics should be used or not */));
        w.push_back(info.format.columns_per_frame);
        h.push_back(info.format.pixels_per_column);
        lidar_data.push_back(std::vector<LidarData>{});
    }
}

void LidarUtil::printInfo(int lidar_index){
    std::cerr << "Sensor " << sensor_info[lidar_index].sn << " Information:" << std::endl;

    size_t w = sensor_info[lidar_index].format.columns_per_frame;
    size_t h = sensor_info[lidar_index].format.pixels_per_column;

    ouster::sensor::ColumnWindow column_window = sensor_info[lidar_index].format.column_window;

    std::cerr << "  Firmware version:  " << sensor_info[lidar_index].image_rev
                << "\n  hostname:          " << hostnames[lidar_index]
                << "\n  Serial number:     " << sensor_info[lidar_index].sn
                << "\n  Product line:      " << sensor_info[lidar_index].prod_line
                << "\n  Scan dimensions:   " << w << " x " << h
                << "\n  Column window:     [" << column_window.first << ", "
                << column_window.second << "]" << std::endl;
}

bool LidarUtil::wholeScan(ouster::sensor::SensorScanSource& source){

    if(lidar_data[0].size() > 2)
        lidarDataClear(0); 

    std::pair<int, std::unique_ptr<ouster::LidarScan>> result = source.get_scan();
    // auto& scan = *result.second;

    ouster::LidarScan& scan = *result.second;
    auto index = result.first;

    if(!result.second) return false;

    auto cloud = ouster::cartesian(scan, luts[index]);
    bool reflect_flag = isFieldAvailable(scan, ouster::sensor::ChanField::REFLECTIVITY);

    Eigen::Array<uint32_t, -1, -1, Eigen::RowMajor> reflectivity;
    Eigen::Array<uint32_t, -1, -1, Eigen::RowMajor> reflectivity_destaggered;
    // if(reflect_flag){
        reflectivity = scan.field<uint8_t>(ouster::sensor::ChanField::REFLECTIVITY).cast<uint32_t>();   
        reflectivity_destaggered = ouster::destagger<uint32_t>(reflectivity, sensor_info[index].format.pixel_shift_by_row);
    // }

    for(int i=0; i<cloud.rows(); i++){
        LidarData data;
        auto xyz = cloud.row(i);
        data.x = xyz(0);
        data.y = xyz(1);
        data.z = xyz(2);
    
        // if(reflect_flag)
            // data.reflectivity = reflectivity_destaggered(i/w[index], i%w[index]);
            data.reflectivity = reflectivity(i/w[index], i%w[index]);
        // if(reflect_flag) data.reflectivity = reflectivity(i, 0);
        
        lidar_data[index].push_back(data);
    } 

    return true;
}

bool LidarUtil::isFieldAvailable(ouster::LidarScan& scan, ouster::sensor::cf_type _field){
            
    try{
        scan.field<uint32_t>(_field);
        return true;
    }catch(const std::invalid_argument&){
        return false;
    }
}

void LidarUtil::lidarDataClear(int index){
    lidar_data[index].clear();
    std::vector <LidarData>().swap(lidar_data[index]);
}

void LidarUtil::recordAll(std::string filename, int mode){

    if(mode == 0){
        std::ofstream file(filename);
        file << "frame_id,x,y,z,reflectivity\n";
        for(int i=0; i<lidar_data.size(); i++){
            for(auto data : lidar_data[i]){
                file << i << "," << data.x << "," 
                << data.y << "," << data.z << "," 
                << data.reflectivity <<"\n";
            }
        }
        file.close();
    }else if(mode == 1){
        std::ofstream file(filename, std::ios::app);
        for(int i=0; i<lidar_data.size(); i++){
            for(auto data : lidar_data[i]){
                file << i << "," << data.x << "," 
                << data.y << "," << data.z << "," 
                << data.reflectivity <<"\n";
            }
        }
        file.close();
    }
    
}

void LidarUtil::recordFrame(std::ofstream& file, int frame_index, std::vector<LidarData> ldata){

    for(auto data : ldata){
        file << frame_index << "," << data.x << "," << data.y << "," 
            << data.z << "," << data.reflectivity << "\n";
    }
}