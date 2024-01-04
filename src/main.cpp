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
#include <opencv2/core/eigen.hpp>

#include "CCalibrator.h"
#include "CTargetDetector.h"

using namespace std;

class WrongPathException: public std::exception{
    public:
        const char* what(){
            return "Image path is wrong or number of images is lower than six";
        }
};

template <typename T>
vector<T> split(string str, char Delimiter) {
    istringstream iss(str);             // istringstream에 str을 담는다.
    string buffer;                      // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
 
    vector<T> result;
 
    // istringstream은 istream을 상속받으므로 getline을 사용할 수 있다.
    while (getline(iss, buffer, Delimiter)) {
        std::stringstream value(buffer);
        T d;
        value>>d;
        result.push_back(d);               // 절삭된 문자열을 vector에 저장
    }
 
    return result;
}

void test_calibration(string img_dir, string type, int mode, int n_x, int n_y, int n_d, double r, double distance, bool is_thermal, bool save_pose){

    vector<string> imgs;

    for (const auto & file : std::filesystem::directory_iterator(img_dir)){
        string s = file.path();
        string path = split<string>(s,'/').back();
        if (path.find(type) != string::npos && path.find(".png") != string::npos){
            imgs.push_back(s);
        }
    }
    if(imgs.size()<6){
        throw WrongPathException();
    }

    sort(imgs.begin(),imgs.end());
    int max_scene = imgs.size();

    TargetDetector detector(n_x, n_y,is_thermal);
    pair<bool,vector<cv::Point2f>> result;
    int count=0;
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene);

    for(int i=0; i<max_scene;i++){
        string path = imgs[i];
        cv::Mat img, img2;
        if (is_thermal){
            img2 = cv::imread(path, cv::IMREAD_GRAYSCALE);
            cv::bilateralFilter(img2,img,-1,10,10);
        }
        else{
            img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        }
        
        if(img.rows == 0){
            throw exception();
        }
        cout<<"start detect: "<<path<<endl;
        result = detector.detect(img, type);
        if(result.first){
            calibrator.inputTarget(result.second);
        }
        else cout<<path<<": detection failed"<<endl;

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
    // int n_x = 4;
    // int n_y= 3;
    // string img_dir= "../imgs/";
    // double r = 0.035; 
    // double distance = 0.09; 


    int n_x = atoi(argv[1]);
    int n_y = atoi(argv[2]);
    string img_dir(argv[3]);
    double r  = atof(argv[4]);
    double distance  = atof(argv[5]);
    bool is_thermal= (1== atoi(argv[6])); //0: rgb, 1:theraml


    cout<<img_dir<<endl;
    printf("%d, %d, %f, %f \n",n_x, n_y, r, distance);

    string type = "circle";
    int n_d= 4;
    bool save_pose = false;
    int mode = 0;
    test_calibration(img_dir,type,mode, n_x, n_y, n_d, r,distance,is_thermal,save_pose);
    finish = clock();
    double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf("%f초\n", duration);

    return 0;
};



