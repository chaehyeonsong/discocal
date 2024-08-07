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
#include "CImagehandler.h"

using namespace std;

class DeficientImagesException: public std::exception{
    public:
        const char* what(){
            return "Image path is wrong or the number of images is lower than six";
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

void do_calibration(string img_dir, string type, int n_x, int n_y, int n_d, double r, double distance, bool check_detection_result, bool save_pose){

    vector<string> imgs;

    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        string path = split<string>(s,'/').back();
        if (path.find(".png") != string::npos || path.find(".PNG") != string::npos || path.find(".jpeg") != string::npos || path.find(".jpg") != string::npos){
            imgs.push_back(s);
        }
    }

    sort(imgs.begin(),imgs.end());
    int max_scene = imgs.size();

    TargetDetector detector(n_x, n_y,check_detection_result);
    pair<bool,vector<cv::Point2f>> result;
    int count=0;
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene);

    
    int width, height;
    Params final_params;
    for(int i=0; i<max_scene;i++){
        string path = imgs[i];
        cv::Mat bgr_img, gray_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        height = bgr_img.rows;
        width = bgr_img.cols;

        gray_img = TargetDetector::preprocessing(bgr_img);
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


    if(type=="circle"){
        final_params=calibrator.calibrate(0, width, height);
        if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_c0.txt");
    }
    else{
        final_params=calibrator.calibrate(2, width, height);
        if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_s.txt");
    }


    Imagehandler imagehandler(width, height,final_params, n_d); 
    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        cv::Mat bgr_img = cv::imread(s, cv::IMREAD_COLOR);
        cv::Mat ud_img = imagehandler.undistort(bgr_img);
        string path = split<string>(s,'/').back();
        string ud_img_path = "../results/"+path;
        cv::imwrite(ud_img_path,ud_img);
    }
    cout<<"undistorted images are saved in ./results folder"<<endl;
}

int main(int argc, char** argv){

    clock_t start, finish;
    start = clock();

    int n_x, n_y, n_d;
    string img_dir;
    double r, distance;
    double fov=-1;
    // user parameter example
    if(argc ==7){
        n_x = atoi(argv[1]);
        n_y = atoi(argv[2]);
        n_d = atoi(argv[3]);
        img_dir = string(argv[4]);
        r  = atof(argv[5]);
        distance  = atof(argv[6]);
    }
    else if (argc ==1){
        n_x = 4;
        n_y= 3;
        n_d = 4;
        img_dir= "../imgs/temp/";
        r = 0.1; 
        distance = 0.27;
    }
    else{
        cout<< "wrong number of arguments"<<endl;
    }

    cout<<img_dir<<endl;
    printf("%d, %d, %d, %f, %f \n",n_x, n_y, n_d, r, distance);

    string type = "circle";
    
    bool check_detection_result = true;
    bool save_pose = false;
    do_calibration(img_dir,type, n_x, n_y, n_d, r,distance,check_detection_result,save_pose);
    finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf("%fsec\n", duration);

    return 0;
};



