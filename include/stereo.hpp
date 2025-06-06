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

using namespace std;

typedef pair<se3,bool> ext;

class StereoCalibration{
public:
    StereoCalibration(){

    };
    ~StereoCalibration(){

    };

    // template <typename T>
    // vector<T> split(string str, char Delimiter);

    vector<pair<se3,bool>> calExtrinsic(YAML::Node args, int camera_index);

    void stereo_calibration(YAML::Node node);
};


vector<pair<se3,bool>> StereoCalibration::calExtrinsic(YAML::Node args, int camera_index){    


    YAML::Node camera_node = args["cameras"][camera_index];
    string img_dir = camera_node["img_dir"].as<string>();
    if(img_dir.back()!='/') img_dir = img_dir+"/";
    int n_d = camera_node["n_d"].as<int>();
    bool cal_intrinsic = false;
    if(camera_node["cal_intrinsic"]) cal_intrinsic= camera_node["cal_intrinsic"].as<bool>();

    string detection_mode="";
    if(camera_node["detection_mode"]){
        detection_mode = camera_node["detection_mode"].as<string>();
    }
    string type = camera_node["type"].as<string>();
    int n_x = camera_node["n_x"].as<int>();
    int n_y = camera_node["n_y"].as<int>();
    double r = camera_node["radius"].as<double>();
    double distance = camera_node["distance"].as<double>();

    
    YAML::Node option_node = args["options"];
    bool visualize = false;
    if(option_node["visualize"]) visualize= option_node["visualize"].as<bool>();
    bool save_pose = false;
    if( option_node["save_pose"]) save_pose = option_node["save_pose"].as<bool>();
    bool save_rpe= false;
    if(option_node["save_rpe"]) save_rpe= option_node["save_rpe"].as<bool>();
    bool evaluation= false;
    if(option_node["evaluation"]) evaluation= option_node["evaluation"].as<bool>();


    vector<string> imgs;

    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        string path = split<string>(s,'/').back();
        if(check_img_path(path)){
            imgs.push_back(s);
        }
    }
    sort(imgs.begin(),imgs.end());
    int max_scene=imgs.size();
    if( cal_intrinsic && max_scene<4+n_d){
        std::cout<<LackOfImageError().what()<<std::endl;
        throw LackOfImageError();
    }
    string results_path = img_dir+"calibration_results/";
    mkdir(results_path);

    TargetDetector detector(n_x, n_y,visualize);
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene,results_path);
    vector<bool> valid_scene; 

    struct timeval  tv;
    double begin, end;
    gettimeofday(&tv, NULL);
    begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    const int LEN = 20;
    const char bar = '=';
    const char blank = ' ';
    bool need_init = true;
    vector<int> fail_img_list;
    for(int i=0; i<imgs.size();i++){
        if(i==max_scene) break;
        string path = imgs[i];
        cv::Mat bgr_img, gray_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);
        if(gray_img.rows == 0) throw exception();
        if(need_init) {
            calibrator.set_image_size(gray_img.rows, gray_img.cols);
            need_init=false;
        }
        if(visualize) cout<<"start detect: "<<path<<endl;
        pair<bool,vector<Shape>> result = detector.detect(gray_img, type);
        detector.save_result(results_path+std::to_string(i)+".png");
        valid_scene.push_back(result.first);
        if(result.first){
            calibrator.inputTarget(result.second);
            print_process(i+1,max_scene,"Detecting image: ");
        }
        else {
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

    // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
    int mode = 0;
    if(type != "circle") mode=2;
    if(cal_intrinsic) calibrator.calibrate(mode,evaluation);
    else calibrator.update_Es(Params(camera_node),mode);

    if(save_pose) calibrator.save_extrinsic();

    vector<se3> target_poses = calibrator.get_extrinsic();
    vector<ext> final_results;
    int count=0;
    for(int i=0; i<valid_scene.size();i++){
        final_results.push_back(ext(target_poses[count],valid_scene[i]));
        if(valid_scene[i]) count++;
    }

    return final_results;
}

void StereoCalibration::stereo_calibration(YAML::Node node){
    bool details=false;
    string path="../";
    if(node["options"]["save_results_path"]) path=node["options"]["save_results_path"].as<string>();
    path = path +"extrinsic_result.txt";
    std::ofstream writeFile(path.data());
    bool save_results = writeFile.is_open();

    int n_camera = node["cameras"].size();
    // if(n_camera<2){
    //     throw WrongTypeException();
    // }

    vector<vector<ext>> all_ext;
    vector<vector<ext>> all_rel;
    for(int i=0; i<n_camera;i++){
        all_ext.push_back(calExtrinsic(node, i));
    }

    vector<ext> extrinsics;
    vector<ext> dest = all_ext[0];
    int n_scene = dest.size();
    
    for(int i=1; i<n_camera;i++){
        if(details && save_results){
            writeFile<<"camera0to"+to_string(i)+"\n";
        }
        vector<ext> origin = all_ext[i];
        vector<ext> rels;
        se3 mean_rel;
        int count=0;
        for(int j=0;j<n_scene;j++){
            bool both_valid = origin[j].second && dest[j].second;
            se3 s_o2d;
            if(both_valid){
                Eigen::Matrix4d T_o2d = LieAlgebra::to_SE3(dest[j].first)*LieAlgebra::to_SE3(origin[j].first).inverse();
                s_o2d = LieAlgebra::to_se3(T_o2d);

                if(details && save_results)writeFile<< to_string(j)+"scene:\t"+s_o2d.to_string()+"\n"; 

                mean_rel.rot += s_o2d.rot;
                mean_rel.trans += s_o2d.trans;
                count++;
            }
            else if(details && save_results) writeFile<< to_string(j)+"scene:\tfail\n"; 
            rels.push_back(ext(s_o2d, both_valid));
        }
        all_rel.push_back(rels);
        if(count>0){
            mean_rel.rot = mean_rel.rot / count;
            mean_rel.trans = mean_rel.trans / count;
            extrinsics.push_back(ext(mean_rel,true));
        }
        else extrinsics.push_back(ext(se3() ,false));
    }

    cout<< "r_x\tr_y\tr_z\tx\ty\tz"<<endl;
    for(int i=0; i<n_camera-1;i++){
        if(extrinsics[i].second){
            se3 E = extrinsics[i].first;
            string message = "camera0to"+to_string(i+1)+": fail\n"; 
            if(extrinsics[i].second){
                message = "camera0to"+to_string(i+1)+":\t"+E.to_string()+"\n"; 
            }
            cout << message;
            if(save_results && writeFile.is_open()){
                writeFile<<message; 
            }
        }
    }
    cout<< "Extrinsic calibration result is saved at :"<< path << endl;
    writeFile.close();

}

#endif