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

typedef pair<cv::Moments,std::vector<cv::Point>> circle_info;

class WrongTypeException: public std::exception{
    public:
        const char* what(){
            return "Type is wrong";
        }
};

class TargetDetector{
    public:
        TargetDetector(int n_x, int n_y, bool draw = true);
        pair<bool,vector<cv::Point2f>>detect(cv::Mat img, string type);
        static cv::Mat preprocessing(cv::Mat img);

    private:
        bool draw;
        int n_x, n_y;
        int size_threshold;
        float drawing_scale;
        double fullfill_threshold, eccentricity_threshold;
        std::vector<cv::Scalar> text_colors;
        
        bool detect_circles(cv::Mat img, vector<cv::Point2f>&target, bool debug=false);
        bool ellipse_test(const cv::Moments &moments);
        void sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist);
        static bool circle_compare(circle_info circle1,  circle_info circle2);
        std::vector<int> get_randomSample(int range, int n);
};
#endif