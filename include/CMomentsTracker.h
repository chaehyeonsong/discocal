#ifndef __CMOMENTSTRACKER_H__
#define __CMOMENTSTRACKER_H__

#include <vector>
#include <array>
#include <iostream>
#include "math.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "utils.h"
using namespace std;


class MomentsTrackerError: public std::exception{
    public:
        const char* what(){
            return "Value exceed the range";
        }
};

class MomentsTracker
{
private:
    /* data */
    int max_dim, max_n, comb_max;
    vector<vector<int>> comb_buffer;
    vector<vector<double>> moment_buffer;
    void init();
    double _M_2i2j(int i, int j);
    double C0n(int n, vector<double> ds);
    double C1n(int n, vector<double> ds);
    double nCr(int i, int j);
    double I_2i2j(int i, int j);
    double M_2i2j(int i, int j);
    double M_2i2j(int i, int j, double a, double b);
    array<double ,3> intSn(int n,double a, double b, double tx, double ty,double alpha);
    

public:
    MomentsTracker(int dim);
    MomentsTracker();
    ~MomentsTracker();


    Point ne2dp(Eigen::Matrix3d Q, vector<double> ds);
    Point wc2dp_Numerical(Eigen::Matrix3d Cw,Eigen::Matrix3d H, vector<double> ds,int total_iter=1600);
    Point ne2dp_Numerical(Eigen::Matrix3d Q, vector<double> ds);

     // 0: moments, 1: conic, 2: points, 3: ne2dp_Numerical, 4: wc2dp_Numerical
    Point project(double wx, double wy, double r,Params intrinsic, Eigen::Matrix3d E, int mode, bool toImage = true);
    array<double, 5> ellipse2array(Eigen::Matrix3d Q);
    Point distort_Point(Point pn, vector<double> ds);
};

















#endif