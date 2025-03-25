#ifndef LIDAR_UTIL_H
#define LIDAR_UTIL_H

#include <string>
#include <vector>
#include <iostream>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

typedef struct lidar_data_pack{
    float x;
    float y;
    float z;
    uint32_t reflectivity;
    //TODO : lidar data - signal,,,,
}LidarData;

class LidarUtil {
public:
    LidarUtil(){

    };
    ~LidarUtil(){

    };

    // add lidar sensor
    // @param hostname : sensor hostname (ip)
    // @param udp_dest : udp destination ip
    // @param udp_port_lidar : udp port for lidar
    // @return void
    void pushHost(const std::string hostname, const std::string udp_dest, const int udp_port_lidar);
    void pushHost(const std::string hostname);
    void pushHost(const std::string hostname, const ouster::sensor::sensor_config config);
    
    // get sensor source
    ouster::sensor::SensorScanSource* getSensorSource();

    // get sensor info
    void getSensorInfo(ouster::sensor::SensorScanSource& source);

    // print info
    void printInfo(int lidar_index);

    // scan
    bool wholeScan(ouster::sensor::SensorScanSource& source);
    // void monoScan(ouster::sensor::SensorScanSource* source, int lidar_index);

    void lidarDataClear(int index);

    bool isFieldAvailable(ouster::LidarScan& scan, const char* _field);

// private:
    std::vector<ouster::sensor::Sensor> sensors;

    // sensor info : sensor information array
    // [usage]
    // sensor_info[lidar index].format.columns_per_frame => get columns per frame
    // sensor_info[lidar index].format.pixels_per_column => get pixels per column
    // sensor_info[lidar index].format.column_window.first => get column window first
    // sensor_info[lidar index].format.column_window.second => get column window second
    // sensor_info[lidar index].image_rev => get firmware version
    // sensor_info[lidar index].sn => get serial number
    // sensor_info[lidar index].prod_line => get product line
    // sensor_info[lidar index].format.udp_profile_lidar => get lidar profile
    std::vector<ouster::sensor::sensor_info> sensor_info;
    std::vector<ouster::XYZLut> luts;
    std::vector<std::string> hostnames;
    std::vector<std::vector<LidarData>> lidar_data;
    std::vector<int> w;
    std::vector<int> h;
    
};

#endif // LIDAR_UTIL_H