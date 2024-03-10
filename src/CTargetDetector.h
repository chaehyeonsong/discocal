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
        

        bool detect_circles(cv::Mat img, vector<cv::Point2f>&target);
        void dfs(cv::Mat img, vector<vector<bool>> &buffer, vector<array<int,3>> &area, int x, int y);
        bool check_pixel(cv::Mat img, int x, int y);
        bool ellipse_test(const vector<array<int,3>> &area,cv::Point2f &pt);
        cv::Point2f area_center(vector<array<int,3>> &area);
        void sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist);
        static bool compare(cv::Point2i a, cv::Point2i b);
};
#endif