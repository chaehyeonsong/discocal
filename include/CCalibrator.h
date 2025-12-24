#ifndef __CCALIBRATOR_H__
#define __CCALIBRATOR_H__

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


class Calibrator{
    public:
        Calibrator(int n_x, int n_y, bool asymmetric, int n_d,double r, double distance, int max_scene, string result_path);
        ~Calibrator() = default;

        void init();
        void inputTarget(vector<Shape> target);
        
        bool cal_initial_params(Params* inital_params);
        Params calibrate(int mode, bool save_jacob = false);
        
        void printParams(Params p);
        int get_num_scene();
        void save_extrinsic(string root_dir);
        void save_extrinsic();
        vector<se3> get_extrinsic();
        void update_Es(Params inital_params, int mode);

        Params calibrate_test(int mode,Params initial_params);
        
        void set_image_size(int _width, int _height);
        // for exp
        void visualize_rep(string path, Params params, int mode);
        // Params calibrate();

        // Eigen::Matrix3d get_extrinsic(Params params, int target_index);
        // Params batch_optimize(std::vector<int> sample, Params initial_params,int phase, double ratio);

    private:
        string root_dir;
        int height, width;
        int num_scene;
        int max_scene;
        int n_x, n_y, n_d;
        bool is_asymmetric;
        double distance;
        double original_r;
        // double curr_r;
        // vector<double> rs;
        double thr_homography;

        vector<se3> Es; // [R1, R2, t] se3: rot , trans
        vector<pair<Eigen::Matrix3d,double>> Hs;

        vector<vector<Shape>> targets;
        vector<vector<Shape>> ud_targets;
        // vector<Eigen::Matrix3d> origin_conics;
        vector<Shape> origin_target;
        array<double, 4> ori_ms; //original_mean_sigma


        Params batch_optimize(std::vector<int> sample, Params initial_params, int mode, bool save_jacob = false, bool fix_intrinsic=false); //unc 반영
        double cal_reprojection_error(std::vector<int> sample,Params params, int mode);
        double cal_calibration_quality(std::vector<int> sample,Params params, int mode);
        void save_data_for_gpr(std::vector<int> sample,Params params, int mode);
        void set_origin_target();
        void normalize_Es();

        //jang's
        void set_inital_Es(Params params);
        array<double, 4> get_mean_sigma(const vector<Shape> &target);
        Eigen::VectorXd get_Vij(const Eigen::Matrix3d &H, int i, int j);
        pair<Eigen::Matrix3d,double> get_H(const vector<Shape> &target); // unc 반영
        Eigen::VectorXd get_absolute_conic();
        Eigen::MatrixXd normalize_matrix(Eigen::MatrixXd matrix);
};


#endif