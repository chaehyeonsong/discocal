#ifndef __CIMAGEHANDLER_H__
#define __CIMAGEHANDLER_H__

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "utils.h"

using namespace std;


class Imagehandler
{
private:
    int width;
    int height;
    int n_d;
    Params params;
    cv::Mat mapx, mapy;

    /* data */
public:
    Imagehandler(int width, int height, Params params, int n_d);
    
    void init();

    cv::Mat undistort(cv::Mat image);

    ~Imagehandler();
};


#endif