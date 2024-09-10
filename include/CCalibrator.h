#ifndef __CCALIBRATOR_H__
#define __CCALIBRATOR_H__

#include <math.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "CMomentsTracker.h"
#include "CLieAlgebra.h"
#include "CTargetDetector.h"
#include "utils.h"

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

class CalibratorError: public std::exception{
    public:
        const char* what(){
            return "Value exceed the range";
        }
};

struct CalibrationFunctor {
    CalibrationFunctor(const vector<Shape> & origin_target,const vector<Shape>& target, int n_d, int mode, bool use_weight=false){
        this->origin_target = origin_target;
        this->target = target;
        this->mode = mode;
        this-> use_weight = use_weight;
        this->tracker = new MomentsTracker(n_d);
    }

    template <typename T>
    bool operator()(const T* const p_radius,const T* const fs,const T* const cs, const T* const p_skew, const T* const distorsion, const T* const rot, const T* const trans,T* residual) const {
        for(int i=0;i<origin_target.size();i++){
            double wx = origin_target[i].x;
            double wy = origin_target[i].y;
            // double radius = 0.05;
            Params params = {fs[0],fs[1],cs[0],cs[1], *p_skew, distorsion[0],distorsion[1],distorsion[2],distorsion[3]};
            Eigen::Matrix3d E, E_inv, Qn;
            Eigen::Vector3d rot_vector{rot[0],rot[1],rot[2]};            
            E= LieAlgebra::to_E(se3(rot_vector,Eigen::Vector3d(trans[0],trans[1],trans[2])));

            Point p_i = tracker->project(wx, wy, *p_radius, params, E,mode);
            double u_e = p_i.x;
            double v_e = p_i.y;

            double u_o = target[i].x;
            double v_o = target[i].y;

            double i_x = 1;
            double i_y = 1;
            if(use_weight){
                i_x = sqrt(target[i].Kxx);
                i_y = sqrt(target[i].Kyy);
                // i_x = sqrt(target[i].area);
                // i_y = sqrt(target[i].area);
            }

            residual[2*i]= (u_o-u_e)*i_x;
            residual[2*i+1]= (v_o-v_e)*i_y;                
        }
        
        return true;
    }
    private:
        int mode;
        // double distance_ratio,radius_ratio;
        bool use_weight;
        vector<Shape> origin_target;
        vector<Shape> target;
        MomentsTracker* tracker;
        // bool normalize;
};

class Calibrator{
    public:
        Calibrator(int n_x, int n_y, int n_d,double r, double distance, int max_scene, string result_path);
        ~Calibrator() = default;

        void init();
        void inputImage(cv::Mat img);
        void inputTarget(vector<Shape> target);
        
        bool cal_initial_params(Params* inital_params);
        Params calibrate(int mode, bool fix_radius=true, bool save_jacob = false);
        
        void printParams(Params p, bool full);
        int get_num_scene();
        void save_extrinsic(string root_dir);
        void save_extrinsic();
        vector<se3> get_extrinsic();
        void update_Es(Params inital_params, int mode, bool fix_radius=true, bool save_jacob= false);

        Params calibrate_test(int mode,Params initial_params);
        

        

        // for exp
        void batch_calibrate(int mode);
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
        double distance;
        double original_r;
        double curr_r;
        // vector<double> rs;
        double thr_homography;

        vector<se3> Es; // [R1, R2, t] se3: rot , trans
        vector<pair<Eigen::Matrix3d,double>> Hs;

        vector<cv::Mat> images;

        vector<vector<Shape>> targets;
        vector<vector<Shape>> ud_targets;
        // vector<Eigen::Matrix3d> origin_conics;
        vector<Shape> origin_target;
        array<double, 4> ori_ms; //original_mean_sigma


        Params batch_optimize(std::vector<int> sample, Params initial_params, int mode, bool fix_r=true, bool save_jacob = false, bool fix_intrinsic=false); //unc 반영
        double cal_reprojection_error(std::vector<int> sample,Params params, int mode);
        void set_origin_target();
        void normalize_Es();

        //jang's
        void set_inital_Es(Params params);
        array<double, 4> get_mean_sigma(const vector<Shape> &target);
        Eigen::VectorXd get_Vij(const Eigen::Matrix3d &H, int i, int j);
        pair<Eigen::Matrix3d,double> get_H(const vector<Shape> &target); // unc 반영
        Eigen::VectorXd get_absolute_conic();
        Eigen::MatrixXd normalize_matrix(Eigen::MatrixXd matrix);
        

        // for exp
        void update_control_point(std::vector<int> real_sample, Params results);
};




#endif