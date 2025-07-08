
#pragma once

#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string>

#include "camera.h"
#include "lidar_util.h"
#include "configs.h"
#include "vis_util.h"

#if CV_VIEW
#include "opencv2/opencv.hpp"
#endif

void setNonBlocking(bool enable);

int main(int argc, char* argv[]) {

    std::vector<std::string> hostname;
    std::string savefile;
    if(argc<3){
        std::cout << "Usage : ./record_example [savefile.csv] [hostname0] [hostname1] ..." << std::endl;
        return -1;
    }

    //get save filename
    savefile = argv[1];

    // get hostnames
    for(int i=2; i<argc; i++){
        hostname.push_back(argv[i]);
    }

    // lidar init
    int lidar_index = 0;
    LidarUtil _lidar;
    for(auto &host: hostname){
        _lidar.pushHost(host); // register lidar
    }

    ouster::sensor::SensorScanSource source(_lidar.sensors);
    _lidar.getSensorInfo(source);
    _lidar.printInfo(lidar_index); // print lidar info : not necessary

    setNonBlocking(true);

    std::cout << "q 입력 시 종료 scanning...\n";
    std::string buffer;

    //savefile open
    std::ofstream csv_file(savefile);

    int frame_index = 0;
    while (true) {
        
        // input detect
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                if (buffer == "q" || buffer == "exit") break;
                buffer.clear();
            } else {
                buffer += c;
                std::cout << c << std::flush;
            }
        }

        //record lidar
        if(_lidar.wholeScan(source)){
            _lidar.recordFrame(csv_file, frame_index, *(_lidar.lidar_data.end()-1));
            frame_index++;
        }
        
    }

    csv_file.close();
    setNonBlocking(false);  // 원래 상태 복구
    std::cout << "\n[Done] lidar data save to " << savefile << "\n";
    return 0;


}

void setNonBlocking(bool enable) {
    static termios oldt;
    static bool initialized = false;

    if (!initialized) {
        tcgetattr(STDIN_FILENO, &oldt);
        initialized = true;
    }

    termios newt = oldt;
    if (enable) {
        newt.c_lflag &= ~(ICANON | ECHO); // 즉시 입력, 입력 문자 표시 안 함
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // non-blocking 모드
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, 0);
    }
}