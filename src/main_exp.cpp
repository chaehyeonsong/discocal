#include <iostream>
#include "time.h"
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <sstream>
#include <yaml-cpp/yaml.h>

#include "CMomentsTracker.h"
#include "CCalibrator.h"
#include "CTargetDetector.h"
#include "CLieAlgebra.h"
#include "utils.h"
using namespace std;



void print_process(int count, int MAX, string prefix){
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

void robust_optimization(YAML::Node node){
    // argparsing
    YAML::Node option_node =node["options"];
    int max_scene= option_node["max_scene"].as<int>();
    bool visualize = option_node["visualize"].as<bool>();
    bool save_pose = option_node["save_pose"].as<bool>();
    bool save_rep= option_node["save_rep"].as<bool>();
    bool save_jacob= option_node["save_jacob"].as<bool>();
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

    // only for exp
    YAML::Node exp_node =node["exp"];
    int sigma= exp_node["gaussian_blur"].as<int>();
    double translation= exp_node["translation"].as<double>();
    double rotation= exp_node["rotation"].as<double>();
   
   

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

    TargetDetector detector(n_x, n_y,visualize);
    pair<bool,vector<Shape>> result;
    int count=0;
    Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene,img_dir);

    // vector<int> success_img_list;
    vector<int> fail_img_list;
    vector<int> translation_blur_imgs;
    vector<int> rotation_blur_imgs;
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
        cv::Mat bgr_img, gray_img,hsv_img, gray_img2;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);
        // cv::cvtColor(bgr_img,gray_img,cv::COLOR_BGR2GRAY);
        // if(sigma>0) {
        //     cv::GaussianBlur(gray_img,gray_img,cv::Size(0, 0), sigma); // blur
        // }
        // if(i<100){
        //     cv::Mat gray_img2;
        //     gray_img2=TargetDetector::translation_blur(gray_img,translation);
        //     gray_img = gray_img2;
        //     translation_blur_imgs.push_back(i);
        // }
        // else if(i<16){
        //     cv::Mat gray_img2;
        //     gray_img2=TargetDetector::rotation_blur(gray_img,rotation);
        //     gray_img = gray_img2;
        //     rotation_blur_imgs.push_back(i);
        // }
        
        // cv::imwrite("../results/"+std::to_string(i)+".png",gray_img);
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
    end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
    double duration =(end - begin) / 1000;
    printf("], Runtime: %.2fs\n", duration);
    

    cout << "translation_blur img list: [";
    for(int i: translation_blur_imgs){
        cout  << i<<" ";
    }
    cout << "]" << endl;
    cout << "rotation_blur img list: [";
    for(int i: rotation_blur_imgs){
        cout  << i<<" ";
    }
    cout << "]" << endl;
    Params final_params;

    // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
    if(type=="circle"){
        int iter = 30;
        int batch_scene=6;
        calibrator.batch_calibrate(0 ,batch_scene,iter);
        // Params intrinsic(1099.759888,	1101.274469,	1045.806842,	786.933921,	1.26357	,-0.275895	,0.053869);
        // calibrator.update_Es(intrinsic, 0,false);

        // if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_c0.txt");
        // if(save_rep) calibrator.visualize_rep(img_dir+"rep_c0.txt",final_params,0);

    }

}

void calibration(YAML::Node node){
    // argparsing
    YAML::Node option_node =node["options"];
    int max_scene= option_node["max_scene"].as<int>();
    bool visualize = option_node["visualize"].as<bool>();
    bool save_pose = option_node["save_pose"].as<bool>();
    bool save_rep= option_node["save_rep"].as<bool>();
    bool save_jacob= option_node["save_jacob"].as<bool>();
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

    // only for exp
    YAML::Node exp_node =node["exp"];
    int sigma= exp_node["gaussian_blur"].as<int>();
    double translation= exp_node["translation"].as<double>();
    double rotation= exp_node["rotation"].as<double>();
   

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
        string path = imgs[i];
        // img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        cv::Mat bgr_img, gray_img;
        bgr_img = cv::imread(path, cv::IMREAD_COLOR);
        
        gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);

        if(sigma>0) {
            cv::GaussianBlur(gray_img,gray_img,cv::Size(0, 0), sigma); // blur
        }

        // if(i<imgs.size()/2){
        //     cv::Mat gray_img2;
        //     gray_img2 = TargetDetector::preprocessing(bgr_img,detection_mode);
        //     gray_img=TargetDetector::synthesize_motion_blur(gray_img2,translation);
        // }

        // if(i<imgs.size()/2){
        //     cv::Mat gray_img2;
        //     gray_img2 = TargetDetector::preprocessing(bgr_img,detection_mode);
        //     gray_img=TargetDetector::rotation_blur(gray_img2,rotation);
        //     cv::imwrite("../rotation.png",gray_img);
        // }
        
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
        final_params=calibrator.calibrate(0,save_jacob);

        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_c0.txt");
        if(save_rep) calibrator.visualize_rep(img_dir+"rep_c0.txt",final_params,0);

    }
    else{
        final_params=calibrator.calibrate(2,save_jacob);
        if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_s.txt");
        if(save_rep) calibrator.visualize_rep(img_dir+"rep_s.txt",final_params,2);

    }
}

int main(int argc, char** argv){
    // omp_set_nested(1);
    // clock_t start, finish;
    // start = clock();

    // YAML::Node node = YAML::LoadFile("../config/exp.yaml");
    // robust_optimization(node);
    string yaml_path="../config/exp.yaml";
    if(argc ==2){
        yaml_path = string(argv[1]);
    }
    cout<<"yaml path: "<<yaml_path<<endl;

    YAML::Node node = YAML::LoadFile(yaml_path);
    // calibration(node);
    robust_optimization(node);

    return 0;
}

// template <typename T>
// vector<T> split(string str, char Delimiter) {
//     istringstream iss(str);             // istringstream에 str을 담는다.
//     string buffer;                      // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
 
//     vector<T> result;
 
//     // istringstream은 istream을 상속받으므로 getline을 사용할 수 있다.
//     while (getline(iss, buffer, Delimiter)) {
//         std::stringstream value(buffer);
//         T d;
//         value>>d;
//         result.push_back(d);               // 절삭된 문자열을 vector에 저장
//     }
 
//     return result;
// }



// // compare rpe using near iamges
// void test_calibration2(YAML::Node node){
//     // argparsing
//     YAML::Node camera_node = node["camera"];
//     string img_dir = camera_node["img_dir"].as<string>();
//     int n_d = camera_node["n_d"].as<int>();
//     string detection_mode = camera_node["detection_mode"].as<string>();

//     int n_x = node["target"]["n_x"].as<int>();
//     int n_y = node["target"]["n_y"].as<int>();
//     string type = node["target"]["type"].as<string>();
//     double r = node["target"]["radius"].as<double>();
//     double distance;
//     if(type == "circle") distance = node["target"]["c_distance"].as<double>();
//     else distance = node["target"]["s_distance"].as<double>();
//     int sigma=node["options"]["sigma"].as<int>();
//     bool visualize = node["options"]["visualize"].as<bool>();
//     // bool save_pose = node["options"]["save_pose"].as<bool>();
//     // bool save_rep= node["options"]["save_rep"].as<bool>();
//     // end parsing

//     vector<string> imgs;

//     for (const auto & file : std::filesystem::directory_iterator(img_dir)){
//         string s = file.path();
//         string path = split<string>(s,'/').back();
//         if (path.find(type) != string::npos  && path.find(".png") != string::npos){
//             imgs.push_back(s);
//         }
//     }
//     sort(imgs.begin(),imgs.end());
//     int max_scene = imgs.size();

//     TargetDetector detector(n_x, n_y,visualize);
//     pair<bool,vector<Shape>> result;
//     int count=0;
//     int n_train=16;
//     int n_test = 8;
//     Calibrator calibrator1 = Calibrator(n_x,n_y,n_d,r,distance,n_train,img_dir);
//     Calibrator calibrator2 = Calibrator(n_x,n_y,n_d,r,distance,n_test,img_dir);

//     for(int i=0; i<max_scene;i++){
//         if(i==imgs.size()) break;

//         string path = imgs[i];
//         cv::Mat bgr_img, gray_img;
//         bgr_img = cv::imread(path, cv::IMREAD_COLOR);
//         gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);
//         if(sigma>0) {
//             cv::GaussianBlur(gray_img,gray_img,cv::Size(0, 0), sigma); // blur
//         }
        
//         if(gray_img.rows == 0){
//             throw exception();
//         }
//         cout<<"start detect: "<<path<<endl;
//         result = detector.detect(gray_img, type);
//         if(result.first){
//             if(i<n_train) calibrator1.inputTarget(result.second);
//             else calibrator2.inputTarget(result.second);

//         }
//         else cout<<path<<": detection failed"<<endl;

//     }
//     Params near_params;
//     if(type=="circle"){
//         near_params= calibrator1.calibrate(0);
//         calibrator2.update_Es(near_params,0);

//         near_params= calibrator1.calibrate(1);
//         calibrator2.update_Es(near_params,1);

//         near_params= calibrator1.calibrate(2);
//         calibrator2.update_Es(near_params,2);
//     }
//     else{
//         near_params= calibrator1.calibrate(2);
//         calibrator2.update_Es(near_params,2);
//     }

// }


// double point_errer(Point p, Point q){
//     return sqrt(pow(p.x-q.x,2)+pow(p.y-q.y,2));
// }


// //-----------start_test_unbised---------------//
// array<double,6> cal_error_of_target(std::vector<Shape> c_keypoints,std::vector<Shape> s_keypoints, double r, double distance, Eigen::Matrix3d E,vector<double> ds){
//     int n_x= 4;
//     int n_y= 3;
//     int n_total = n_x* n_y;
//     double fx{600}, fy{600}, cx{600}, cy{450};
//     std::vector<Eigen::Matrix3d> origin_conics;
//     std::vector<Point> origin_points;
//     for(int j=0;j<n_y;j++){
//         for(int i=0;i<n_x;i++){
//             double cx,cy;
//             cx = distance*(i+1);
//             cy = distance*(n_y-j);
//             Eigen::Matrix3d oc;
//             oc <<   1.0, 0.0, -cx,
//                     0.0, 1.0, -cy,
//                     -cx, -cy, pow(cx,2)+pow(cy,2)-pow(r,2);
//             origin_conics.push_back(oc);
//             origin_points.push_back(Point(cx,cy));
//         }
//     }

//     MomentsTracker tracker = MomentsTracker(2);
//     array<double,6> all_rpe={0,};
//     // double allrpe0{0}, allrpe1{0}, allrpe2{0},allrpe4{0};
//     for(int i=0; i<n_total;i++){
//         Eigen::Matrix3d Qn = E.inverse().transpose() * origin_conics[i] * E.inverse();
//         // Eigen::Vector3d p{0,0,1};
//         // p = origin_conics[i].inverse()*p;
//         Eigen::Vector3d p{origin_points[i].x,origin_points[i].y,1};
//         p = E*p;
//         Point d_point0 =  tracker.ne2dp(Qn,ds); // mode 0
//         array<double, 5> ellipse = tracker.ellipse2array(Qn);
//         Point n_point(ellipse[0],ellipse[1]);
//         Point d_point1 = tracker.distort_Point(n_point,ds); // mode 1
//         Point d_point2 = tracker.distort_Point(Point(p(0)/p(2),p(1)/p(2)),ds);  // mode 2
//         Point d_point3 = tracker.wc2dp_Numerical(origin_conics[i],E,ds,900);  // mode 3
//         Point d_point4 = tracker.wc2dp_Numerical(origin_conics[i],E,ds,1600);  // mode 4

//         double rpe0 = sqrt(pow(d_point0.x*fx+cx - c_keypoints.at(i).x, 2)+ pow(d_point0.y*fy+cy-c_keypoints.at(i).y,2));
//         double rpe1 = sqrt(pow(d_point1.x*fx+cx - c_keypoints.at(i).x, 2)+ pow(d_point1.y*fy+cy-c_keypoints.at(i).y,2));
//         double rpe2 = sqrt(pow(d_point2.x*fx+cx - c_keypoints.at(i).x, 2)+ pow(d_point2.y*fy+cy-c_keypoints.at(i).y,2));
//         double rpe3 = sqrt(pow(d_point3.x*fx+cx - c_keypoints.at(i).x, 2)+ pow(d_point3.y*fy+cy-c_keypoints.at(i).y,2));
//         double rpe4 = sqrt(pow(d_point4.x*fx+cx - c_keypoints.at(i).x, 2)+ pow(d_point4.y*fy+cy-c_keypoints.at(i).y,2));
//         double rpe10 = sqrt(pow(d_point2.x*fx+cx - s_keypoints.at(i).x, 2)+ pow(d_point2.y*fy+cy-s_keypoints.at(i).y,2));
//         all_rpe[0] += rpe0;
//         all_rpe[1] += rpe1;
//         all_rpe[2] += rpe2;
//         all_rpe[3] += rpe3;
//         all_rpe[4] += rpe4;
//         all_rpe[5] += rpe10;

//     }
//     for(int i=0; i<all_rpe.size();i++){
//         all_rpe[i] = all_rpe[i]/n_total;
//     }
//     return all_rpe;

// }

// pair<array<double ,6>,array<double ,6>> rpe_error(string img_dir, vector<Eigen::Matrix3d> Es, int n_x, int n_y,int sigma,double distance ,double r, double d1){
//     double d2 = -0.25*d1;
//     vector<string> circle_imgs;
//     vector<string> square_imgs;
//     TargetDetector detector(n_x, n_y,false);
//     pair<bool,vector<Shape>> c_result, s_result;
//     // int num_mode=5
//     array<double,6> means{0,};
//     array<double,6> stds{0,};
//     int n=0;

//     for (const auto & file : std::filesystem::directory_iterator(img_dir)){
//         string s = file.path();
//         if (s.find("circle") != string::npos){
//             circle_imgs.push_back(s);
//         }
//         if (s.find("square") != string::npos){
//             square_imgs.push_back(s);
//         }
//     }
//     sort(circle_imgs.begin(),circle_imgs.end());
//     sort(square_imgs.begin(),square_imgs.end());


//     for(int i=0; i<circle_imgs.size();i++){
//         string c_path = circle_imgs[i];
//         string s_path = square_imgs[i];
//         cv::Mat c_img = cv::imread(c_path, cv::IMREAD_GRAYSCALE);
//         cv::Mat s_img = cv::imread(s_path, cv::IMREAD_GRAYSCALE);
//         if(sigma>0) {
//             cv::GaussianBlur(c_img,c_img,cv::Size(0, 0), sigma); // blur
//             cv::GaussianBlur(s_img,s_img,cv::Size(0, 0), sigma);
//         }
//         // cout<<"start detect: "<<path<<endl;
//         c_result = detector.detect(c_img,"circle");
//         s_result = detector.detect(s_img,"square");
//         if (c_result.first && s_result.first){
//             std::vector<double> ds{1, d1, d2};
//             auto errors = cal_error_of_target(c_result.second,s_result.second,r, distance, Es.at(i),ds);
//             // printf("d1: %f, r:%f -> e0: %f, e1: %f, e2: %f\n", d1,r, errors[0], errors[1],errors[2]);
//             for(int k=0;k<6;k++){
//                 means[k] += errors[k];
//                 stds[k] += pow(errors[k],2);
//             }

//             // errors = cal_error_of_target(s_result.second,r, distance, Es.at(i),ds);
//             // means[4] += errors[2];
//             // stds[4] += pow(errors[2],2);

//             n++;
//         }
//         else cout<<i<<" detection failed"<<endl;
//     }

//     for(int k=0;k<6;k++){
//         means[k] = means[k]/n;
//         stds[k] = sqrt(stds[k]/n - pow(means[k],2));
//     }
//     printf("r: %f, d1:%f -> e0: %f, e1: %f, e2: %f, e3: %f, e4: %f, e10: %f\n", r,d1, means[0], means[1],means[2],means[3],means[4],means[5]);
//     return pair<array<double ,6>,array<double ,6>>{means,stds};


// }


// void test_unbiased(string test_mode, int n_x, int n_y, int sigma, double distance){
//     string mode_dir= "../data/unbiased_exp/"+test_mode+'/';
//     vector<string> dirs;
//     for (const auto & file : std::filesystem::directory_iterator(mode_dir)){
//         string s = file.path();
//         dirs.push_back(s+'/');
//     }
//     sort(dirs.begin(),dirs.end());

//     string es_path =  "../data/unbiased_exp/Es.txt";
//     string line;
//     fstream fs;
//     vector<Eigen::Matrix3d> Es;
//     fs.open(es_path, fstream::in);
//     while (getline(fs, line))
//     {
//         vector<double> tokens = split<double>(line,'\t');
//         Eigen::Matrix3d E;
//         E<< tokens[0],tokens[1],tokens[2],tokens[3],tokens[4],tokens[5],tokens[6],tokens[7],tokens[8];
//         Es.push_back(E);
//     }
//     fs.close();

//     string path =   "../results/ubt_"+test_mode+"_"+to_string(sigma)+".txt";
//     std::ofstream writeFile(path.data());
//     double t_r =0.07;
//     double t_d1 = -0.4;
//     for(int i=0; i<dirs.size();i++){
//         if(test_mode == "radius"){
//             t_r = 0.01+0.006*i;
//         }
//         else{
//             t_d1= -0.4 + 0.08*i;
//         }
//         string img_dir= dirs.at(i);
//         pair<array<double ,6>,array<double ,6>> results= rpe_error(img_dir, Es, n_x, n_y,sigma,distance ,t_r, t_d1);
//         if(writeFile.is_open()){
//             writeFile<<t_r<<"\t"<<t_d1<<"\t"<<results.first[0] <<"\t"<<results.first[1] <<"\t"<<results.first[2]<<"\t"<<results.first[3]<<"\t"<<results.first[4]<<"\t"<<results.first[5]<<"\t"
//             <<results.second[0] <<"\t"<<results.second[1] <<"\t"<<results.second[2]<<"\t"<<results.second[3]<<"\t"<<results.second[4]<<"\t"<<results.second[5]<<"\n";  
//         }      
//     }
//     writeFile.close();
// }
// //-----------end_test_unbised---------------//



// //---------------start  extrinsic test-----------------//
// pair<double, double> compare_pose_error_ones(string gt_path, string est_path, bool fullmode){
//     // string gt_path =  root_dir+"pose.txt";
//     string line;
//     fstream fs;
//     vector<se3> gt_Es;
//     fs.open(gt_path, fstream::in);
//     while (getline(fs, line))
//     {
//         vector<double> tokens = split<double>(line,' ');
//         Eigen::Vector3d trans{tokens[0],tokens[1],tokens[2] };
//         Eigen::Quaterniond quat{tokens[6],tokens[3],tokens[4],tokens[5]}; //w ,x ,y, z
//         // printf("%f %f %f %f\n",quat.x(), quat.y(), quat.z(), quat.w());
//         se3 E(LieAlgebra::to_so3(quat.normalized().toRotationMatrix()), trans);
//         gt_Es.push_back(E);
//     }
//     fs.close();

//     // string est_path =  root_dir+"est_pose_c0.txt";
//     vector<se3> est_Es;
//     fs.open(est_path, fstream::in);
//     while (getline(fs, line))
//     {
//         vector<double> tokens = split<double>(line,'\t');
//         Eigen::Vector3d trans{tokens[0],tokens[1],tokens[2] };
//         Eigen::Vector3d rot{tokens[3],tokens[4],tokens[5]};
//         se3 E(rot, trans);
//         est_Es.push_back(E);
//     }
//     fs.close();

//     // cout<<LieAlgebra::to_SE3(gt_Es[0])<<endl;
//     // cout<<LieAlgebra::to_SE3(gt_Es[0]).inverse()<<endl;
//     // cout<< LieAlgebra::to_SE3(gt_Es[0]) * LieAlgebra::to_SE3(gt_Es[0]).inverse()<<endl;
//     int n = gt_Es.size();
//     if(n != est_Es.size()) printf("------------size error!!!\n----------");
    
//     // Eigen::Matrix4d temp = LieAlgebra::to_SE3(est_Es[0]);
//     // cout<<temp<<endl;
//     double total_trans_error{0.}, total_rot_error{0.};

//     if(fullmode){
//         vector<pair<se3, se3>> ABs ,CDs;
//         for(int i=0; i<n;i++){
//             Eigen::Matrix4d gt_base = LieAlgebra::to_SE3(gt_Es[i]).inverse(); //Tom
//             Eigen::Matrix4d est_base = LieAlgebra::to_SE3(est_Es[i]).inverse(); //Ttc
//             for(int j=i+1;j<n;j++){
//                 Eigen::Matrix4d A= gt_base*LieAlgebra::to_SE3(gt_Es[j]);
//                 Eigen::Matrix4d B= est_base*  LieAlgebra::to_SE3(est_Es[j]);
//                 // Eigen::Matrix4d est_T= temp.inverse()*gt_T*temp;
//                 ABs.push_back(pair<se3, se3>{LieAlgebra::to_se3(A),LieAlgebra::to_se3(B)});

//                 Eigen::Matrix4d C= LieAlgebra::to_SE3(gt_Es[j])*gt_base;
//                 Eigen::Matrix4d D= LieAlgebra::to_SE3(est_Es[j])*est_base;

//                 CDs.push_back(pair<se3, se3>{LieAlgebra::to_se3(C),LieAlgebra::to_se3(D)});
//             }
//         }
//         se3 x = LieAlgebra::solveAXXB(ABs);
//         se3 y = LieAlgebra::solveAXXB(CDs);
        
//         for(int i=0;i<n;i++){
//             se3 gt = gt_Es[i];
//             se3 camera = est_Es[i];
//             Eigen::Matrix4d Est = LieAlgebra::to_SE3(y)*LieAlgebra::to_SE3(camera)*LieAlgebra::to_SE3(x).inverse();
//             se3 est =LieAlgebra::to_se3(Est);
//             pair<double, double> error = LieAlgebra::dist(gt, est);
//             total_rot_error += error.first;
//             total_trans_error += error.second;
//             // printf("%dth->rot_error:%f, trans_error: %f\n",i,error.first, error.second);
//         }
//         total_rot_error = total_rot_error/n;
//         total_trans_error = total_trans_error/n;

//         return pair<double, double>(total_rot_error, total_trans_error);
//     }
//     else{
//         vector<pair<se3, se3>> ABs;
//         // Eigen::Matrix4d temp = LieAlgebra::to_SE3(est_Es[0]);
//         for(int i=0; i<n;i++){
//             Eigen::Matrix4d gt_base = LieAlgebra::to_SE3(gt_Es[i]).inverse(); //Tom
//             Eigen::Matrix4d est_base = LieAlgebra::to_SE3(est_Es[i]).inverse(); //Ttc
//             for(int j=i+1;j<n;j++){
//                 Eigen::Matrix4d gt_T= gt_base*LieAlgebra::to_SE3(gt_Es[j]);
//                 Eigen::Matrix4d est_T= est_base*  LieAlgebra::to_SE3(est_Es[j]);
//                 // Eigen::Matrix4d est_T= temp.inverse()*gt_T*temp;
//                 ABs.push_back(pair<se3, se3>{LieAlgebra::to_se3(gt_T),LieAlgebra::to_se3(est_T)});
//             }
//         }
//         se3 x = LieAlgebra::solveAXXB(ABs);

//         int n_edge = ABs.size();
//         for(int i=0;i<n_edge;i++){
//             se3 a = ABs[i].first;
//             se3 b = ABs[i].second;
//             Eigen::Matrix4d Ahat = LieAlgebra::to_SE3(x)*LieAlgebra::to_SE3(b)*LieAlgebra::to_SE3(x).inverse();
//             se3 ahat =LieAlgebra::to_se3(Ahat);
//             pair<double, double> error = LieAlgebra::dist(a, ahat);
//             total_rot_error += error.first;
//             total_trans_error += error.second;
//         }
//         total_rot_error = total_rot_error/n_edge;
//         total_trans_error = total_trans_error/n_edge;
//         return pair<double, double>(total_rot_error, total_trans_error);
//     }


    
// }


// void compare_pose_error_total(string root_dir, bool fullmode=false){
//     printf("-----------r_error, t_error--------\n");
//     printf("square, ours, conic, point \n");
//     pair<double, double> s_error = compare_pose_error_ones(root_dir+"gt_pose_s.txt",root_dir+"est_pose_s.txt",fullmode);
//     pair<double, double> c_error0 = compare_pose_error_ones(root_dir+"gt_pose_c.txt",root_dir+"est_pose_c0.txt",fullmode);
//     pair<double, double> c_error1 = compare_pose_error_ones(root_dir+"gt_pose_c.txt",root_dir+"est_pose_c1.txt",fullmode);
//     pair<double, double> c_error2 = compare_pose_error_ones(root_dir+"gt_pose_c.txt",root_dir+"est_pose_c2.txt",fullmode);
//     pair<double, double> c_error4 = compare_pose_error_ones(root_dir+"gt_pose_c.txt",root_dir+"est_pose_c4.txt",fullmode);
//     printf("%f, %f\n",s_error.first, s_error.second);
//     printf("%f, %f\n",c_error0.first, c_error0.second);
//     printf("%f, %f\n",c_error1.first, c_error1.second);
//     printf("%f, %f\n",c_error2.first, c_error2.second);
//     printf("%f, %f\n",c_error4.first, c_error4.second);
// }

// void stereo_calibration(string root_dir){
//     string left_ext_path, right_ext_path;
//     left_ext_path = root_dir + "left/est_pose_c0.txt";
//     right_ext_path = root_dir + "right/est_pose_c0.txt";
//     string line;
//     fstream fs;
//     vector<se3> left_Es, right_Es;
//     fs.open(left_ext_path, fstream::in);
//     while (getline(fs, line))
//     {
//         vector<double> tokens = split<double>(line,'\t');
//         Eigen::Vector3d trans{tokens[0],tokens[1],tokens[2] };
//         Eigen::Vector3d rot{tokens[3],tokens[4],tokens[5]};
//         se3 E(rot, trans);
//         left_Es.push_back(E);
//     }
//     fs.close();

//     fs.open(right_ext_path, fstream::in);
//     while (getline(fs, line))
//     {
//         vector<double> tokens = split<double>(line,'\t');
//         Eigen::Vector3d trans{tokens[0],tokens[1],tokens[2] };
//         Eigen::Vector3d rot{tokens[3],tokens[4],tokens[5]};
//         se3 E(rot, trans);
//         right_Es.push_back(E);
//     }
//     fs.close();

//     if(left_Es.size() != right_Es.size()){
//         printf("size is not same!!!\n");
//         throw WrongTypeException();
//     }

//     vector<se3> rels;
//     for(int i=0; i<left_Es.size();i++){
//         Eigen::Matrix4d T_r2l = LieAlgebra::to_SE3(left_Es[i])*LieAlgebra::to_SE3(right_Es[i]).inverse();
//         se3 E_r2l = LieAlgebra::to_se3(T_r2l);
//         rels.push_back(E_r2l);
//     }

//     string path = root_dir+"rel.txt";
//     std::ofstream writeFile(path.data());
//     for(int i=0; i<rels.size();i++){
//         se3 E = rels[i];
//         if(writeFile.is_open()){
//             writeFile<<E.trans(0)<<"\t"<<E.trans(1)<<"\t"<<E.trans(2)<<"\t"<<E.rot(0)<<"\t"<<E.rot(1)<<"\t"<<E.rot(2)<<"\n";  
//         }      
//     }
//     writeFile.close();

// }
// //---------------end extrinsic test-----------------//


// void test_calibration(YAML::Node node){
//     // argparsing
//     YAML::Node camera_node = node["camera"];
//     string img_dir = camera_node["img_dir"].as<string>();
//     int n_d = camera_node["n_d"].as<int>();
//     string detection_mode = camera_node["detection_mode"].as<string>();

//     YAML::Node target_node =node["target"];
//     int n_x = target_node["n_x"].as<int>();
//     int n_y = target_node["n_y"].as<int>();
//     string type = target_node["type"].as<string>();
//     double r = target_node["radius"].as<double>();
//     double distance;
//     if(type == "circle") distance = target_node["c_distance"].as<double>();
//     else distance = target_node["s_distance"].as<double>();
   
//     YAML::Node option_node =node["options"];
//     int max_scene= option_node["max_scene"].as<int>();
//     int sigma= option_node["sigma"].as<int>();
//     bool visualize = option_node["visualize"].as<bool>();
//     bool save_pose = option_node["save_pose"].as<bool>();
//     bool save_rep= option_node["save_rep"].as<bool>();
//     bool save_jacob= option_node["save_jacob"].as<bool>();
//     bool fix_radius= option_node["fix_radius"].as<bool>();
//     // end parsing


//     vector<string> imgs;

//     for (const auto & file : std::filesystem::directory_iterator(img_dir)){
//         string s = file.path();
//         string path = split<string>(s,'/').back();
//         // if (path.find(type) == string::npos) continue;
//         if(path.find(".png") != string::npos || path.find(".PNG") != string::npos || path.find(".jpeg") != string::npos || path.find(".jpg") != string::npos){
//                 imgs.push_back(s);
//         }
//     }
//     sort(imgs.begin(),imgs.end());
//     if(max_scene==0) max_scene=imgs.size();

//     TargetDetector detector(n_x, n_y,visualize);
//     pair<bool,vector<Shape>> result;
//     int count=0;
//     Calibrator calibrator = Calibrator(n_x,n_y,n_d,r,distance,max_scene,img_dir);

//     vector<int> success_img_list;
//     for(int i=0; i<imgs.size();i++){
//         if(calibrator.get_num_scene()==max_scene) break;
//         string path = imgs[i];
//         // img = cv::imread(path, cv::IMREAD_GRAYSCALE);
//         cv::Mat bgr_img, gray_img,hsv_img;
//         bgr_img = cv::imread(path, cv::IMREAD_COLOR);
//         gray_img = TargetDetector::preprocessing(bgr_img,detection_mode);

//         if(sigma>0) {
//             cv::GaussianBlur(gray_img,gray_img,cv::Size(0, 0), sigma); // blur
//         }
        
//         if(gray_img.rows == 0){
//             throw exception();
//         }
//         if(visualize) cout<<"start detect: "<<path<<endl;
//         result = detector.detect(gray_img, type);
//         if(result.first){
//             calibrator.inputTarget(result.second);
//             success_img_list.push_back(i);
//         }
//         else{
//             if(visualize) cout<<path<<": detection failed"<<endl;
//         }

//     }
//     cout << "img_list: ";
//     for(int i: success_img_list){
//         cout <<" " << i;
//     }
//     cout << endl; 
    
//     Params final_params;

//     // 0: moment, 1: conic, 2: point, 4: numerical, 5: iterative
//     if(type=="circle"){
//         // calibrator.batch_calibrate(0);
//         // calibrator.batch_calibrate(1);
//         // calibrator.batch_calibrate(2);
//         // calibrator.batch_calibrate(3);
//         // calibrator.batch_calibrate(4);
//         // calibrator.batch_calibrate(5);


//         final_params=calibrator.calibrate(0,fix_radius,save_jacob);
//         if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_c0.txt");
//         if(save_rep) calibrator.visualize_rep(img_dir+"rep_c0.txt",final_params,0);

//         // final_params=calibrator.calibrate(1);
//         // if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_c1.txt");
//         // if(save_rep) calibrator.visualize_rep(img_dir+"rep_c1.txt",final_params,1);

//         // final_params=calibrator.calibrate(2);
//         // if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_c2.txt");
//         // if(save_rep) calibrator.visualize_rep(img_dir+"rep_c2.txt",final_params,2);

//         // final_params=calibrator.calibrate(4);
//         // if(save_pose) calibrator.get_extrinsic(img_dir+"est_pose_c4.txt");
//         // if(save_rep) calibrator.visualize_rep(img_dir+"rep_c4.txt",final_params,4);
//     }
//     else{
//         // calibrator.batch_calibrate(2);

//         final_params=calibrator.calibrate(2,fix_radius,save_jacob);
//         if(save_pose) calibrator.save_extrinsic(img_dir+"est_pose_s.txt");
//         if(save_rep) calibrator.visualize_rep(img_dir+"rep_s.txt",final_params,2);
//     }

// }