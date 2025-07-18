#ifndef __CMONO_H__
#define __CMONO_H__


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

using namespace std;

class MonoCalibration{
public:
    MonoCalibration(){};
    ~MonoCalibration(){};

    // template <typename T>
    // vector<T> split(string str, char Delimiter);
    void mono_calibration(YAML::Node node);
};


void MonoCalibration::mono_calibration(YAML::Node node){
    // argparsing

    YAML::Node camera_node = node["camera"];
    string img_dir = camera_node["img_dir"].as<string>();
    if(img_dir.back()!='/') img_dir = img_dir+"/";
    int n_d = camera_node["n_d"].as<int>();
    string detection_mode = camera_node["detection_mode"].as<string>();

    int n_x = camera_node["n_x"].as<int>();
    int n_y = camera_node["n_y"].as<int>();
    string type = camera_node["type"].as<string>();
    double r = camera_node["radius"].as<double>();
    double distance = camera_node["distance"].as<double>();


    YAML::Node option_node =node["options"];
    bool visualize = false;
    if(option_node["visualize"]) visualize= option_node["visualize"].as<bool>();
    bool save_pose = false;
    if( option_node["save_pose"]) save_pose = option_node["save_pose"].as<bool>();
    bool save_rpe= false;
    if(option_node["save_rpe"]) save_rpe= option_node["save_rpe"].as<bool>();
    bool save_jacob= false;
    if(option_node["save_jacob"]) save_jacob= option_node["save_jacob"].as<bool>();
    bool fix_intrinsic= false;
    if(option_node["fix_intrinsic"])fix_intrinsic= option_node["fix_intrinsic"].as<bool>();

    // end parsing

    vector<string> imgs;
    cout<<"Image_dir:"<<img_dir<<endl;
    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        string path = split<string>(s,'/').back();
        // if (path.find(type) == string::npos) continue;
        if(check_img_path(path)){
                imgs.push_back(s);
        }
    }
    sort(imgs.begin(),imgs.end());
    int max_scene=imgs.size();
    if(max_scene<4+n_d){
        std::cout<<LackOfImageError().what()<<std::endl;
        throw LackOfImageError();
    }

    string results_path = img_dir+"detection_results/";
    mkdir(results_path);

    TargetDetector detector(n_x, n_y,visualize);
    pair<bool,vector<Shape>> result;
    int count=0;
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene,img_dir);

    // vector<int> success_img_list;
    vector<int> fail_img_list;
    struct timeval  tv;
	double begin, end;
    gettimeofday(&tv, NULL);
	begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    const int LEN = 20;
    const char bar = '=';
    const char blank = ' ';
    for(int i=0; i<imgs.size();i++){
        if(calibrator.get_num_scene()==max_scene) break;
        print_process(i+1,max_scene,"Detecting image: ");
        string path = imgs[i];
        // img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        cv::Mat bgr_img, gray_img,hsv_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);
                
        if(gray_img.rows == 0){
            throw exception();
        }

        if(visualize) cout<<"start detect: "<<path<<endl;
        result = detector.detect(gray_img, type);
        detector.save_result(results_path+std::to_string(i)+".png");
        if(result.first){
            calibrator.inputTarget(result.second);
        }
        else{
            if(visualize) cout<<path<<": detection failed"<<endl;
            fail_img_list.push_back(i);
        }

    }

    cout << "\nDetection fail img list: [";
    for(int i: fail_img_list){
        cout  << i<<" ";
    }
    gettimeofday(&tv, NULL);
    end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    double duration =(end - begin) / 1000;
    printf("], Runtime: %.2fs\n", duration);

    cout << "Detection results are saved at: "+results_path<<endl;
    
    Params final_params;

    // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
    if(type=="circle"){
        if(fix_intrinsic) {
            cout<<"fix intrinsic mode" <<endl;
            final_params = Params(camera_node);
            tabulate::Table table =  final_params.to_table(false, true);
            cout<<table <<endl;
            calibrator.update_Es(final_params,0);
        }
        else final_params=calibrator.calibrate(0,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_c0.txt");
        if(save_rpe) calibrator.visualize_rep(img_dir+"rpe_c0.txt",final_params,0);
    }
    else{
        final_params=calibrator.calibrate(2,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_s.txt");
        if(save_rpe) calibrator.visualize_rep(img_dir+"rpe_s.txt",final_params,2);
    }

}

#endif