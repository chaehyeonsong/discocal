#ifndef __CSTEREO_H__
#define __CSTEREO_H__

#include <iostream>
#include "time.h"
#include <string>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <unistd.h>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "CCalibrator.h"
#include "CTargetDetector.h"
#include "utils.h"
#include "CStereoCalibrator.h"

using namespace std;

class StereoCalibration
{
public:
    StereoCalibration() {

    };
    ~StereoCalibration() {

    };

    // template <typename T>
    // vector<T> split(string str, char Delimiter);

    pair<vector<se3>, vector<Target>> calExtrinsic(YAML::Node args, int camera_index);

    void stereo_calibration(YAML::Node node);
};

pair<vector<se3>, vector<Target>> StereoCalibration::calExtrinsic(YAML::Node args, int camera_index)
{

    YAML::Node camera_node = args["cameras"][camera_index];
    string img_dir = camera_node["img_dir"].as<string>();
    if (img_dir.back() != '/')
        img_dir = img_dir + "/";
    int n_d = camera_node["n_d"].as<int>();

    string detection_mode = "";
    if (camera_node["detection_mode"])
    {
        detection_mode = camera_node["detection_mode"].as<string>();
    }
    string type = camera_node["type"].as<string>();
    int n_x = camera_node["n_x"].as<int>();
    int n_y = camera_node["n_y"].as<int>();
    double r = camera_node["radius"].as<double>();
    double distance = camera_node["distance"].as<double>();
    bool is_asymmetric = false;
    if (camera_node["asymmetric"])
        is_asymmetric = camera_node["asymmetric"].as<bool>();

    YAML::Node option_node = args["options"];
    bool visualize = false;
    if (option_node["visualize"])
        visualize = option_node["visualize"].as<bool>();
    bool save_pose = false;
    if (option_node["save_pose"])
        save_pose = option_node["save_pose"].as<bool>();
    bool save_rpe = false;
    if (option_node["save_rpe"])
        save_rpe = option_node["save_rpe"].as<bool>();
    bool evaluation = false;
    if (option_node["evaluation"])
        evaluation = option_node["evaluation"].as<bool>();

    vector<string> imgs;

    for (const auto &file : std::filesystem::directory_iterator(img_dir))
    {
        string s = file.path();
        string path = split<string>(s, '/').back();
        if (check_img_path(path))
        {
            imgs.push_back(s);
        }
    }
    sort(imgs.begin(), imgs.end());
    int max_scene = imgs.size();
    string results_path = img_dir + "calibration_results/";
    mkdir(results_path);

    TargetDetector detector(n_x, n_y, is_asymmetric, visualize);
    Calibrator calibrator = Calibrator(n_x, n_y, is_asymmetric, n_d, r, distance, max_scene, results_path);
    // vector<bool> valid_scene;

    struct timeval tv;
    double begin, end;
    gettimeofday(&tv, NULL);
    begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
    const int LEN = 20;
    const char bar = '=';
    const char blank = ' ';
    bool need_init = true;
    vector<int> fail_img_list;
    vector<Target> targets;
    for (int i = 0; i < imgs.size(); i++)
    {
        if (i == max_scene)
            break;
        string path = imgs[i];
        cv::Mat bgr_img, gray_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        gray_img = TargetDetector::preprocessing(bgr_img, detection_mode);
        if (gray_img.rows == 0)
            throw exception();
        if (need_init)
        {
            calibrator.set_image_size(gray_img.rows, gray_img.cols);
            need_init = false;
        }
        if (visualize)
            cout << "start detect: " << path << endl;
        Target result = detector.detect(gray_img, type);
        targets.push_back(result);
        detector.save_result(results_path + std::to_string(i) + ".png");
        // valid_scene.push_back(result.first);
        if (result.first)
        {
            calibrator.inputTarget(result.second);
            print_process(i + 1, max_scene, "Detecting image: ");
        }
        else
        {
            if (visualize)
                cout << path << ": detection failed" << endl;
            fail_img_list.push_back(i);
        }
    }
    cout << "\nDetection fail img list: [";
    for (int i : fail_img_list)
    {
        cout << i << " ";
    }

    gettimeofday(&tv, NULL);
    end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
    double duration = (end - begin) / 1000;
    printf("], Runtime: %.2fs\n", duration);
    cout << "Detection results are saved at: " + results_path << endl;

    // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
    int mode = 0;
    if (type != "circle")
        mode = 2;
    calibrator.update_Es(Params(camera_node), mode);

    calibrator.update_Es(Params(camera_node), mode);

    if (save_pose)
        calibrator.save_extrinsic();

    vector<se3> target_poses = calibrator.get_extrinsic();
    vector<se3> final_results;
    int count = 0;
    for (int i = 0; i < targets.size(); i++)
    {
        if (targets[i].first)
        {
            final_results.push_back(target_poses[count]);
            count++;
        }
        else
        {
            final_results.push_back(se3());
        }
    }

    return pair<vector<se3>, vector<Target>>(final_results, targets);
}

void StereoCalibration::stereo_calibration(YAML::Node node)
{
    // bool details = false;
    string results_dir= "./extrinsic_result/";
    if (node["options"]["results_path"])
        results_dir = node["options"]["results_path"].as<string>();
        if(results_dir.back()!='/') results_dir = results_dir+"/";
    mkdir(results_dir);

    int n_camera = node["cameras"].size();
    // if(n_camera<2){
    //     throw WrongTypeException();
    // }
    YAML::Node camera_node = node["cameras"][0];
    int n_x = camera_node["n_x"].as<int>();
    int n_y = camera_node["n_y"].as<int>();
    double distance = camera_node["distance"].as<double>();
    bool is_asymmetric = camera_node["asymmetric"] ? camera_node["asymmetric"].as<bool>() : false;
    bool full_optimize = node["options"]["full_optimize"] ? node["options"]["full_optimize"].as<bool>() : false;

    if(full_optimize){
        cout<<"Full optimization for stereo calibration is enabled. (Both optimize intrinsic and extrinsic parameters)"<<endl;
    }
    StereoCalibrator stereo_calibrator(n_x, n_y, is_asymmetric, distance);

    vector<vector<Target>> all_targets; // [camera][scene]
    vector<vector<se3>> all_ext;
    vector<Params> all_params;
    vector<double> all_radius;
    vector<double> all_n_d;
    for (int i = 0; i < n_camera; i++)
    {
        all_params.push_back(Params(node["cameras"][i]));
        all_radius.push_back(node["cameras"][i]["radius"].as<double>());
        all_n_d.push_back(node["cameras"][i]["n_d"].as<double>());

        auto [calExtrinsic, targets] = this->calExtrinsic(node, i);
        all_targets.push_back(targets);
        all_ext.push_back(calExtrinsic);
    }
    vector<se3> extrinsics;
    vector<se3> dest = all_ext[0];
    vector<bool> success;
    int n_scene = dest.size();

    for (int i = 1; i < n_camera; i++)
    {
        // if (details && save_results)
        // {
        //     writeFile << "camera0to" + to_string(i) + "\n";
        // }
        vector<se3> origin = all_ext[i];
        se3 mean_rel;
        int count = 0;
        for (int j = 0; j < n_scene; j++)
        {
            bool both_valid = all_targets[i][j].first && all_targets[0][j].first;
            se3 s_o2d;
            if (both_valid)
            {
                Eigen::Matrix4d T_o2d = LieAlgebra::to_SE3(dest[j]) * LieAlgebra::to_SE3(origin[j]).inverse();
                s_o2d = LieAlgebra::to_se3(T_o2d);

                // if (details && save_results)
                //     writeFile << to_string(j) + "scene:\t" + s_o2d.to_string() + "\n";

                mean_rel.rot += s_o2d.rot;
                mean_rel.trans += s_o2d.trans;
                count++;
            }
            // else if (details && save_results)
            //     writeFile << to_string(j) + "scene:\tfail\n";
        }
        if (count > 0)
        {
            mean_rel.rot = mean_rel.rot / count;
            mean_rel.trans = mean_rel.trans / count;
            extrinsics.push_back(mean_rel);
            success.push_back(true);
        }
        else
        {
            extrinsics.push_back(se3());
            success.push_back(false);
        }
    }

    stereo_calibrator.calibrate(all_targets, dest, extrinsics, all_params, all_radius, all_n_d, full_optimize);
    for(int i=0;i<all_params.size();i++){
        string intrinsic_path = results_dir + "camera"+ to_string(i)+"_intrinsic.yaml";
        std::ofstream intrinsicFile(intrinsic_path.data());
        Params final_params = all_params[i];
        YAML::Node camera_node = node["cameras"][i];
        if (!intrinsicFile.is_open()) {
            cout << "Unable to open file " << intrinsic_path << endl;
            continue;
        }
        intrinsicFile<<"img_dir: \""<<camera_node["img_dir"].as<string>() <<"\"\n";  
        intrinsicFile<<"n_d: "<<camera_node["n_d"].as<int>() <<"\n";  
        intrinsicFile<<"radius: " << camera_node["radius"].as<double>() << "\n";
        intrinsicFile<<"distance: " << camera_node["distance"].as<double>() << "\n";

        intrinsicFile<<"fx: " << final_params.K[0] << "\n";
        intrinsicFile<<"fy: " << final_params.K[1]<< "\n";
        intrinsicFile<<"cx: " << final_params.K[2] << "\n";
        intrinsicFile<<"cy: " << final_params.K[3] << "\n";
        intrinsicFile<<"skew: " << final_params.K[4] << "\n";

        intrinsicFile<<"d1: " << final_params.d[0] << "\n";
        intrinsicFile<<"d2: " << final_params.d[1] << "\n";
        intrinsicFile<<"d3: " << final_params.d[2] << "\n";
        intrinsicFile<<"d4: " << final_params.d[3] << "\n";

        intrinsicFile<<"sfx: " << final_params.s_K[0] << "\n";
        intrinsicFile<<"sfy: " << final_params.s_K[1] << "\n";
        intrinsicFile<<"scx: " << final_params.s_K[2] << "\n";
        intrinsicFile<<"scy: " << final_params.s_K[3] << "\n";

        intrinsicFile<<"sd1: " << final_params.s_d[0] << "\n";
        intrinsicFile<<"sd2: " << final_params.s_d[1] << "\n";
        intrinsicFile<<"sd3: " << final_params.s_d[2] << "\n";
        intrinsicFile<<"sd4: " << final_params.s_d[3] << "\n";

        intrinsicFile.close();
    }

    string pose_path = results_dir + "relative_camera_poses.txt";
    std::ofstream writeFile(pose_path.data());
    bool save_results = writeFile.is_open();
    string header = "Relative poses described in camera0 coordinates\n";
    cout << header;
    if (writeFile.is_open())
    {
        writeFile << header;
        writeFile << "r_x\t\tr_y\t\tr_z\t\tt_x\t\tt_y\t\tt_z\n";
    }

    tabulate::Table table;
    table.add_row({"Camera Pair","r_x","r_y","r_z","t_x","t_y","t_z"});
    string fail_icon = "---";

    for (int i = 0; i < n_camera - 1; i++)
    {
        se3 E = extrinsics[i];
        string message = "camera0to" + to_string(i + 1) + ": fail\n";
        if (success[i])
        {
            message = "camera0to" + to_string(i + 1) + ":\t" + E.to_string() + "\n";
            table.add_row({"camera0to" + to_string(i + 1), to_stringf(E.rot[0], 4), to_stringf(E.rot[1], 4), to_stringf(E.rot[2], 4), to_stringf(E.trans[0], 4), to_stringf(E.trans[1], 4), to_stringf(E.trans[2], 4)});
        }
        else table.add_row({"camera0to" + to_string(i + 1), "fail", fail_icon, fail_icon, fail_icon, fail_icon, fail_icon});

        if (writeFile.is_open())
        {
            writeFile << message;
        }
    }
    cout<<table <<endl;
    writeFile.close();
    cout << "Extrinsic calibration result is saved at :" << results_dir << endl;
    
}

#endif