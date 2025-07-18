#ifndef __CTARGETDETECTOR_H__
#define __CTARGETDETECTOR_H__

#include <iostream>
#include <string.h>
#include <algorithm>
#include <omp.h>
#include <sys/time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "CCircleGridFinder.hpp"
#include "utils.h"


using namespace std;

typedef pair<Shape,std::vector<cv::Point>> circle_info;

class WrongTypeException: public std::exception{
    public:
        const char* what(){
            return "Type is wrong";
        }
};

class TargetDetector{
    public:
        TargetDetector(int n_x, int n_y, bool draw = true);
        pair<bool,vector<Shape>>detect(cv::Mat& img, string type, bool key_input=true); //modified: key_input
        static cv::Mat preprocessing(cv::Mat img, string detection_mode);
        static cv::Mat translation_blur(const cv::Mat &img, double trans);
        static cv::Mat rotation_blur(const cv::Mat &img, double dtheta);
        void save_result(string path);

    private:
        double numerical_stable;
        bool draw;
        cv::Mat detection_result;
        int n_x, n_y;
        int size_threshold;
        float drawing_scale;
        double fullfill_threshold, eccentricity_threshold, distance_threshold;
        std::vector<cv::Scalar> text_colors;
        array<double,3> cov2ellipse(double Kxx, double Kxy, double Kyy);
        int intensity_range(const  cv::Mat &gray_img, int x, int y, float step_x, float step_y, int step_length);
        int intensity_range(const  cv::Mat &gray_img, int x, int y, int window_size);
        Shape contour2shape(const vector<cv::Point2i> &contour);
        bool cal_shape_cov(const vector<cv::Point2i> &contour, Shape* shape,  const  cv::Mat &gray_img, const  cv::Mat &grad_x3, const cv::Mat &grad_y3);
        bool detect_circles(cv::Mat& input_img, cv::Mat& output_img, vector<Shape>&target, bool debug=false);
        bool ellipse_test(const Shape &shape);
        void sortTarget(vector<cv::Point2f>&source, vector<cv::Point2f>&dist);
        static bool circle_compare(circle_info circle1,  circle_info circle2);

        void update_autocorrelation(cv::Mat &src, vector<Shape>& control_shapes);

        void visualize_result(cv::Mat& img_output, pair<bool,vector<Shape>> &result, bool key_input = true); //modified: key_input

        //for exp
        bool detect_circles_banilar(cv::Mat& img, cv::Mat& img_output,vector<Shape>&target, bool debug);
        
};
#endif