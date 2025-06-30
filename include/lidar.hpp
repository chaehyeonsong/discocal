#ifndef __CLIDAR_H__
#define __CLIDAR_H__

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include "stereo.hpp"      
#include "utils.h"           // se3, LieAlgebra µî
#include "Lidarparam.h"     

using namespace std;


#include "CLidarextrinsic.h"
typedef pair<se3, bool> ext;
class LidarCalibration {
public:
    LidarCalibration() {}
    ~LidarCalibration() {}

    void lidar_camera_calibration(YAML::Node args);
    vector<ext> calExtrinsic_lidar(YAML::Node args);
};

vector<ext> LidarCalibration::calExtrinsic_lidar(YAML::Node args) {
    vector<ext> results;

    YAML::Node lidar_node = args["lidar"][0];
    YAML::Node option_node = args["options"];

    LidarCalibrationParams params;
    params.point_cloud_dir = lidar_node["point_cloud_dir"].as<string>();
    if (params.point_cloud_dir.back() != '/') params.point_cloud_dir += "/";
    params.detection_mode = lidar_node["detection_mode"].as<string>();
    params.type = lidar_node["type"].as<string>();
    params.n_x = lidar_node["n_x"].as<int>();
    params.n_y = lidar_node["n_y"].as<int>();
    params.radius = lidar_node["radius"].as<double>();
    params.distance = lidar_node["distance"].as<double>();
    params.visualize = option_node["visualize"].as<bool>();
    printf("Lidar Calibration Parameters:\n");
    params.coarse_bd_x = {lidar_node["coarse_bd_x1"].as<double>(), lidar_node["coarse_bd_x2"].as<double>()};
    params.coarse_bd_y = {lidar_node["coarse_bd_y1"].as<double>(), lidar_node["coarse_bd_y2"].as<double>()};
    params.coarse_bd_z = {lidar_node["coarse_bd_z1"].as<double>(), lidar_node["coarse_bd_z2"].as<double>()};
    params.eps = lidar_node["eps"].as<float>();
    params.distance_threshold = lidar_node["distance_threshold"].as<double>();
    params.max_iterations = lidar_node["max_iterations"].as<int>();
    params.cdr = lidar_node["cdr"].as<double>();
    params.direction_var = lidar_node["direction_var"].as<double>();
    printf("parsed params:\n");
    vector<string> cloud_files;
    for (const auto& file : std::filesystem::directory_iterator(params.point_cloud_dir)) {
        string path = file.path();
        if (path.ends_with(".pcd") || path.ends_with(".ply")) {
            cloud_files.push_back(path);
        }
    }
    sort(cloud_files.begin(), cloud_files.end());

    for (const auto& pcd_path : cloud_files) {
        se3 pose;
        bool success = estimate_lidar_extrinsic(pcd_path, params, pose);
        results.push_back({pose, success});
    }
    cout<< "Lidar extrinsic calibration completed. Number of scenes: " << results.size() << endl;
    for (const auto& result : results) {
        cout << "Scene " << &result - &results[0] << ": " << (result.second ? "Success" : "Failure") << endl;
    }
    return results;
}

void LidarCalibration::lidar_camera_calibration(YAML::Node args) {
    bool details = false;
    string path = "../";
    if (args["options"]["save_results_path"])
        path = args["options"]["save_results_path"].as<string>();
    path = path + "lidar_camera_extrinsic.txt";

    std::ofstream writeFile(path.data());
    bool save_results = writeFile.is_open();

    // 1. LiDAR extrinsic
    vector<ext> lidar_exts = calExtrinsic_lidar(args);
    if (save_results) writeFile << "[LiDAR extrinsic results]\n";
    for (size_t i = 0; i < lidar_exts.size(); ++i) {
        if (!lidar_exts[i].second) continue;
        if (save_results) writeFile << "[Scene " << i << "]\n" 
                                    << lidar_exts[i].first.to_string() << "\n\n";
    }
    // 2. Camera extrinsic
    StereoCalibration stereo;
    cout<< "Camera extrinsic calibration started...\n";
    vector<ext> cam_exts = stereo.calExtrinsic(args, 0);
    if (save_results) writeFile << "[Camera extrinsic results]\n";
    for (size_t i = 0; i < cam_exts.size(); ++i) {
        if (!cam_exts[i].second) continue;
        if (save_results) writeFile << "[Scene " << i << "]\n" 
                                    << cam_exts[i].first.to_string() << "\n\n";
    }
    // 3. Compute LiDAR to Camera transformation per scene
    std::vector<se3> rels;
    for (size_t i = 0; i < std::min(lidar_exts.size(), cam_exts.size()); ++i) {
        if (!lidar_exts[i].second || !cam_exts[i].second) continue;

        Eigen::Matrix4d T_cam = LieAlgebra::to_SE3(cam_exts[i].first);
        Eigen::Matrix4d T_lidar = LieAlgebra::to_SE3(lidar_exts[i].first);
        Eigen::Matrix4d T_rel = T_cam * T_lidar.inverse();

        se3 rel = LieAlgebra::to_se3(T_rel);
        rels.push_back(rel);

        if (details && save_results) {
            writeFile << "[Scene " << i << "]\n";
            writeFile << rel.to_string() << "\n\n";
        }
    }
    if(args["options"]["visualize"].as<bool>()) {
        for (size_t i = 0; i < cam_exts.size(); ++i) {
            if (!cam_exts[i].second) continue;

            Eigen::Matrix4d T_cam = LieAlgebra::to_SE3(cam_exts[i].first);
            std::cout << "[INFO] Camera SE(3) for Scene " << i << ":\n" << T_cam << std::endl;
            Eigen::Matrix4d T_lidar = LieAlgebra::to_SE3(lidar_exts[i].first);
            std::cout << "[INFO] LiDAR SE(3) for Scene " << i << ":\n" << T_lidar << std::endl;
            std::cout << "[INFO] rels " << i << ":\n" << LieAlgebra::to_SE3(rels[i]) << std::endl;

        }
    }

    // 4. Average the transformations
    se3 result;
    if (!rels.empty()) {
        for (const auto& r : rels) {
            result.rot += r.rot;
            result.trans += r.trans;
        }
        result.rot /= rels.size();
        result.trans /= rels.size();
    }
    std::cout << "\n\nResult...\n" << "-----------------------------------------------------------------------------------------\n";
    std::cout << "LiDAR to Camera extrinsic:\n" << result.to_string() << std::endl;
    if (save_results) writeFile << "[LiDAR to Camera]\n" << result.to_string() << std::endl;
    Eigen::Matrix4d T_SE3 = LieAlgebra::to_SE3(result);
    std::cout << "\nLiDAR to Camera SE(3) matrix:\n" << T_SE3 << std::endl;

    if (save_results) writeFile << "\n[LiDAR to Camera SE3]\n" << T_SE3 << "\n";
    writeFile.close();
}

#endif