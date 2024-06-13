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
    
    
    
public:
    MomentsTracker(int dim);
    MomentsTracker();
    ~MomentsTracker();

    double I_2i2j(int i, int j);
    
    double M_2i2j(int i, int j);
    double M_2i2j(int i, int j, double a, double b);
    array<double ,3> intSn(int n,double a, double b, double tx, double ty,double alpha );
    Point ne2dp(Eigen::Matrix3d Q, vector<double> ds);
    Point distort_Point(Point pn, vector<double> ds);
    double nCr(int i, int j);

    array<double, 5> ellipse2array(Eigen::Matrix3d Q);
};

















#endif