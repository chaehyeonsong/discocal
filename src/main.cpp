#include <iostream>
#include "time.h"
#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "CCalibrator.h"
#include "CTargetDetector.h"

using namespace std;

class DeficientImagesException: public std::exception{
    public:
        const char* what(){
            return "Image path is wrong or number of images is lower than six";
        }
};

template <typename T>
vector<T> split(string str, char Delimiter) {
    istringstream iss(str);             
    string buffer;                      
    vector<T> result;
    while (getline(iss, buffer, Delimiter)) {
        std::stringstream value(buffer);
        T d;
        value>>d;
        result.push_back(d);               
    }
    return result;
}

void do_calibration(string img_dir, string type, int mode, int n_x, int n_y, int n_d, double r, double distance, bool check_detection_result, bool save_pose){

    vector<string> imgs;

    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        string path = split<string>(s,'/').back();
        if (path.find(type) != string::npos && path.find(".png") != string::npos){
            imgs.push_back(s);
        }
    }

    sort(imgs.begin(),imgs.end());
    int max_scene = imgs.size();

    TargetDetector detector(n_x, n_y,check_detection_result);
    pair<bool,vector<cv::Point2f>> result;
    int count=0;
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene);

    for(int i=0; i<max_scene;i++){
        string path = imgs[i];
        cv::Mat bgr_img, gray_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        cv::fastNlMeansDenoisingColored(bgr_img, bgr_img, 10, 10, 7, 21);
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
        if(gray_img.rows == 0){
            throw exception();
        }
        cout<<"start detect: "<<path<<endl;
        result = detector.detect(gray_img, type);
        if(result.first){
            calibrator.inputTarget(result.second);

        }
        else cout<<path<<": detection failed"<<endl;

    }
    if(calibrator.get_num_scene()<6){
        throw DeficientImagesException();
    }

    Params final_params;

    if(type=="circle"){
        final_params=calibrator.calibrate(0);
        if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_c0.txt");
    }
    else{
        final_params=calibrator.calibrate(2);
        if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_s.txt");
    }
};


int main(int argc, char** argv){

    clock_t start, finish;
    start = clock();
    // user parameter
    int n_x = 4;
    int n_y= 3;
    int n_d = 3;
    string img_dir= "../imgs/circle_0/";
    double r = 0.035; 
    double distance = 0.09; 


    // int n_x = atoi(argv[1]);
    // int n_y = atoi(argv[2]);
    // int n_d = atoi(argv[3]);
    // string img_dir(argv[4]);
    // double r  = atof(argv[5]);
    // double distance  = atof(argv[6]);


    cout<<img_dir<<endl;
    printf("%d, %d, %d, %f, %f \n",n_x, n_y, n_d, r, distance);

    string type = "circle";
    
    bool check_detection_result = true;
    bool save_pose = false;
    int mode = 0;
    do_calibration(img_dir,type,mode, n_x, n_y, n_d, r,distance,check_detection_result,save_pose);
    finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf("%f초\n", duration);

    return 0;
};



