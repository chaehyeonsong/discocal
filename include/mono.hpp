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

    void print_process(int count, int MAX, string prefix="");

    void mono_calibration(YAML::Node node);
};

// template <typename T>
// vector<T> MonoCalibration::split(string str, char Delimiter) {
//     istringstream iss(str);
//     string buffer;

//     vector<T> result;

//     while (getline(iss, buffer, Delimiter)) {
//         std::stringstream value(buffer);
//         T d;
//         value >> d;
//         result.push_back(d);
//     }

//     return result;
// }

void MonoCalibration::print_process(int count, int MAX, string prefix){
    const char bar = '='; // 프로그레스바 문자  
	const char blank = ' '; // 비어있는 프로그레스바 문자  
	// const int MAX = 20; // 프로그레스바 길이  
	int i; // 반복문 전용 변수  
	int bar_count; // 프로그레스바 갯수 저장 변수  
	float percent; // 퍼센트 저장 변수  
    printf("\r%s%d/%d [", prefix.c_str(),count, MAX); // 진행 상태 출력  
    percent = (float)count/MAX*100; // 퍼센트 계산  
    bar_count = count; // 프로그레스바 갯수 계산  
    for(i=0; i<MAX; i++) { // LEN길이의 프로그레스바 출력  
        if(bar_count > i) { // 프로그레스바 길이보다 i가 작으면 
            printf("%c", bar);
        } else { // i가 더 커지면  
            printf("%c", blank);
        }
    }
    printf("] %0.2f%%", percent); // 퍼센트 출력  
    cout<<flush;
}

void MonoCalibration::mono_calibration(YAML::Node node){
    // argparsing
    YAML::Node option_node =node["options"];
    int max_scene= option_node["max_scene"].as<int>();
    bool visualize = option_node["visualize"].as<bool>();
    float visualize_scale = option_node["visualize_scale"].as<float>();
    bool save_pose = option_node["save_pose"].as<bool>();
    bool save_rep= option_node["save_rep"].as<bool>();
    bool save_jacob= option_node["save_jacob"].as<bool>();
    // bool consider_skew= option_node["consider_skew"].as<bool>();
    bool fix_intrinsic= option_node["fix_intrinsic"].as<bool>();

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
   

    // end parsing

    vector<string> imgs;
    cout<<"Image_dir:"<<img_dir<<endl;
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

    TargetDetector detector(n_x, n_y,visualize,visualize_scale);
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
        
        if(result.first){
            calibrator.inputTarget(result.second);
            
            print_process(i+1,max_scene,"Detecting image: ");

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
    
    Params final_params;

    // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
    if(type=="circle"){
        if(fix_intrinsic) {
            cout<<"fix intrinsic mode" <<endl;
            final_params = Params(camera_node);
            tabulate::Table table =  final_params.to_table(false, true);
            cout<<table <<endl;
            calibrator.update_Es(final_params,0,save_jacob);
        }
        else final_params=calibrator.calibrate(0,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_c0.txt");
        if(save_rep) calibrator.visualize_rep(img_dir+"rep_c0.txt",final_params,0);
    }
    else{
        final_params=calibrator.calibrate(2,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_s.txt");
        if(save_rep) calibrator.visualize_rep(img_dir+"rep_s.txt",final_params,2);
    }

}

#endif