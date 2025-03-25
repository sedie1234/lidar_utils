#include <string>
#include <vector>
#include <iostream>

#include "ouster/client.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#include "ouster/sensor_scan_source.h"
#include "ouster/types.h"

#include "lidar_util.h"

int main(int argc, char* argv[]){

    std::string hostname = argv[1];

    LidarUtil _lidar;

    // lidar init : lidar start
    _lidar.pushHost(hostname);
    // ouster::sensor::SensorScanSource* source = _lidar.getSensorSource();
    ouster::sensor::SensorScanSource source(_lidar.sensors);
    _lidar.getSensorInfo(source);

    // print lidar info
    _lidar.printInfo(0);

    // read lidar scan data
    while(!_lidar.wholeScan(source));
    
    // check field data
    // std::cout << _lidar.lidar_data[0][0] << std::endl;
    for(int i=0; i<_lidar.lidar_data[0].size(); i++){
        if(_lidar.lidar_data[0][i].x != 0){
        
            printf("%f %f %f %d\n", _lidar.lidar_data[0][i].x,
                                    _lidar.lidar_data[0][i].y,
                                    _lidar.lidar_data[0][i].z,
                                    _lidar.lidar_data[0][i].reflectivity);    
        }
    }
    
    
    return 0;
}
