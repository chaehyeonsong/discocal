// lidar_params.hpp
#pragma once
#include <string>

struct LidarCalibrationParams {
    std::string point_cloud_dir;
    std::string detection_mode;
    std::string type;
    int n_x = 0, n_y = 0;
    double radius = 0.0;
    double distance = 0.0;
    bool visualize = false;

    std::pair<double, double> coarse_bd_x = {0.0, 10.0};  // coarse boundary box x
    std::pair<double, double> coarse_bd_y = {-5.0, 5.0};  // coarse boundary box y
    std::pair<double, double> coarse_bd_z = {-1.0, 3.0};  // coarse boundary box z
    float eps = 0.05;  // DBSCAN eps
    double distance_threshold = 0.025;  // RANSAC distance threshold
    int max_iterations = 10000;  // RANSAC max iterations
    double cdr = 0.42;  // centroid distance ratio threshold
    double direction_var = 1.0;  // direction variance threshold
};
