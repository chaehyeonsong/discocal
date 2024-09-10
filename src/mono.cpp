#include <iostream>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <yaml-cpp/yaml.h>

#include "time.h"
#include "CMomentsTracker.h"
#include "CCalibrator.h"
#include "CTargetDetector.h"
#include "CLieAlgebra.h"
#include "utils.h"

using namespace std;

void do_calibration(YAML::Node node){
// argparsing
    YAML::Node camera_node = node["camera"];
    string img_dir = camera_node["img_dir"].as<string>();
    int n_d = camera_node["n_d"].as<int>();
    string detection_mode = camera_node["detection_mode"].as<string>();

    YAML::Node target_node =node["target"];
    int n_x = target_node["n_x"].as<int>();
    int n_y = target_node["n_y"].as<int>();
    string type = target_node["type"].as<string>();
    double r = target_node["radius"].as<double>();
    double distance;
    if(type == "circle") distance = target_node["c_distance"].as<double>();
    else distance = target_node["s_distance"].as<double>();
   
    YAML::Node option_node =node["options"];
    int max_scene= option_node["max_scene"].as<int>();
    int sigma= option_node["sigma"].as<int>();
    bool visualize = option_node["visualize"].as<bool>();
    bool save_pose = option_node["save_pose"].as<bool>();
    bool save_rep= option_node["save_rep"].as<bool>();
    bool save_jacob= option_node["save_jacob"].as<bool>();
    bool fix_radius= option_node["fix_radius"].as<bool>();
    // end parsing


    vector<string> imgs;

    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        string path = split<string>(s,'/').back();
        // if (path.find(type) == string::npos) continue;
        if(path.find(".png") != string::npos || path.find(".PNG") != string::npos || path.find(".jpeg") != string::npos || path.find(".jpg") != string::npos){
                imgs.push_back(s);
        }
    }
    sort(imgs.begin(),imgs.end());
    if(max_scene==0) max_scene=imgs.size();

    TargetDetector detector(n_x, n_y,visualize);
    pair<bool,vector<Shape>> result;
    int count=0;
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene,img_dir);

    for(int i=0; i<imgs.size();i++){
        if(calibrator.get_num_scene()==max_scene) break;
        string path = imgs[i];
        cv::Mat img, img2;

        // img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        cv::Mat bgr_img, gray_img,hsv_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);

        if(sigma>0) {
            cv::GaussianBlur(gray_img,gray_img,cv::Size(0, 0), sigma); // blur
        }
        
        if(gray_img.rows == 0){
            throw exception();
        }
        cout<<"start detect: "<<path<<endl;
        result = detector.detect(gray_img, type);
        if(result.first){
            calibrator.inputTarget(result.second);
        }
        else{
            cout<<path<<": detection failed"<<endl;
        }

    }
    
    Params final_params;

    // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
    if(type=="circle"){
        final_params=calibrator.calibrate(0,fix_radius,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_c0.txt");
    }
    else{
        final_params=calibrator.calibrate(2,fix_radius,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_s.txt");
    }

}

int main(int argc, char** argv){
    // omp_set_nested(1);
    clock_t start, finish;
    start = clock();

    YAML::Node node = YAML::LoadFile("../config/mono.yaml");
    do_calibration(node);
    finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf("%f초\n", duration);

    return 0;
}

