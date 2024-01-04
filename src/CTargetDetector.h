#ifndef __CTARGETDETECTOR_H__
#define __CTARGETDETECTOR_H__

#include <iostream>
#include <string.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <unistd.h>
#include <stack>
#include <memory>
#include "CCircleGridFinder.hpp"
using namespace std;


// struct Sorter{

//     public:
//         static void setOrigin(double x, double y){
//             ori_x = x;
//             ori_y = y;
//         };
//         static bool sort_theta(Conic &conic1, Conic& conic2){
//             double theta1 = atan2(ori_y-conic1.y,conic1.x-ori_x);
//             if(theta1<0) theta1+=2*CV_PI;
//             double theta2 = atan2(ori_y-conic2.y,conic2.x-ori_x);
//             if(theta2<0) theta2+=2*CV_PI;
//             return theta1 <theta2;
//         };
//         static bool sort_Area(Conic &conic1, Conic &conic2){
//             return conic1.getArea()>conic2.getArea();
//         };
//     private:
//         static double ori_x;
//         static double ori_y;

// };

class WrongTypeException: public std::exception{
    public:
        const char* what(){
            return "Type is wrong";
        }
};

class TargetDetector{
    public:
        TargetDetector(int n_x, int n_y, bool is_thermal= false);
        pair<bool,vector<cv::Point2f>>detect(cv::Mat img, string type);
        static double ALPHA ,DELTA;

    private:
        bool draw,is_thermal;
        int n_x, n_y;
        int color_threshold,color_threshold_max,color_threshold_min, color_threshold_step;
        int size_threshold;
        double fullfill_threshold1,fullfill_threshold2, eccentricity_threshold;
        bool prev_success, use_weight;
        bool do_iterative_search;
        string TYPE;
        // cv::Ptr<cv::SimpleBlobDetector> blob_detector;
        

        bool detect_circles(cv::Mat img, vector<cv::Point2f>&target);
        void dfs(cv::Mat img, vector<vector<bool>> &buffer, vector<array<int,3>> &area, int x, int y);
        bool check_pixel(cv::Mat img, int x, int y);
        bool ellipse_test(const vector<array<int,3>> &area,cv::Point2f &pt);
        cv::Point2f area_center(vector<array<int,3>> &area);
        void sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist);
        // void sortTarget2(vector<cv::Point2f>&source, vector<cv::Point2f>&dist);
        static bool compare(cv::Point2i a, cv::Point2i b);
};

////////////////////////////////////////////////

// class CircleDetector: public cv::Ptr<cv::Feature2D>{
//     public:
//         CircleDetector(int n_x, int n_y);
//         ~CircleDetector(){};
//         void read (const cv::FileNode &);
//         void write (cv::FileStorage &) const;
//         void detect(cv::InputArray image, std::vector<cv::Point2f>& keypoints, cv::InputArray mask=cv::noArray());
//         void compute( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
//         {
//             throw WrongTypeException();
//         }
//         void detectAndCompute( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, const cv::Mat & mask = cv::Mat())
//         {
//             throw WrongTypeException();
//         }
//     private:
//         int n_x, n_y;
//         int color_threshold;
//         int size_threshold;
//         void dfs(cv::InputArray img, vector<vector<bool>> &buffer, vector<array<int,3>> &area, int x, int y);
//         bool check_pixel(cv::InputArray img, int x, int y);
//         bool ellipse_test(const vector<array<int,3>> &area);
//         cv::Point2f area_center(vector<array<int,3>> &area);
// };
#endif