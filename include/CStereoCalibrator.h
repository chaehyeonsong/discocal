#ifndef __CSTEREOCALIBRATOR_H__
#define __CSTEREOCALIBRATOR_H__

#include <math.h>
#include <fstream>
#include <iostream>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "CMomentsTracker.h"
#include "CLieAlgebra.h"
#include "utils.h"
#include "CFunctors.h"

#include <time.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;

using ceres::NumericDiffCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class StereoCalibrator{
    public:
        StereoCalibrator(int n_x, int n_y, bool asymmetric, double distance);
        ~StereoCalibrator() = default;
        
        vector<array<double, 6>> calibrate(
            const vector<vector<Target>> & all_targets,
            vector<se3>& Es,
            vector<se3>& all_ext,
            vector<Params>& all_params,
            const vector<double>& all_radius,
            const vector<double>& all_n_d,
            bool full_optimize
        );
        
    private:
        int n_x, n_y;
        bool is_asymmetric;
        double distance;

        vector<Shape> cal_origin_target(double radius);
        void normalize_Es(vector<se3>& Es);
};

#endif