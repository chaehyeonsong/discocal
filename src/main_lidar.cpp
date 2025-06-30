#include <yaml-cpp/yaml.h>   
#include "lidar.hpp"
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    std::string yaml_path = "../config/lidar.yaml";
    if (argc == 2) {
        yaml_path = std::string(argv[1]);
    }
    std::cout << "YAML path: " << yaml_path << std::endl;

    YAML::Node config = YAML::LoadFile(yaml_path);

    try {
        LidarCalibration calib;
        calib.lidar_camera_calibration(config);  // 단 하나의 함수로 실행
    } catch (const std::exception& e) {
        std::cerr << "Calibration error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
