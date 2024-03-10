#ifndef __CCALIBRATOR_H__
#define __CCALIBRATOR_H__

#include <math.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "CMomentumTracker.h"
#include "CLieAlgebra.h"

using namespace std;

using ceres::NumericDiffCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct Params{
    double fx;
    double fy;
    double cx;
    double cy;
    double skew;
    double d[4];
    double radius;
};

class CalibratorError: public std::exception{
    public:
        const char* what(){
            return "Value exceed the range";
        }
};

struct CalibrationFunctor {
    CalibrationFunctor(const vector<Point> & origin_target,const vector<Point>& target, int n_d, int mode){
        this->origin_target = origin_target;
        this->target = target;
        this->mode = mode;
        this->tracker = new MomentumTracker(n_d);
    }

    template <typename T>
    bool operator()(const T* const p_radius,const T* const fs,const T* const cs, const T* const p_skew, const T* const distorsion, const T* const rot, const T* const trans,T* residual) const {
        for(int i=0;i<origin_target.size();i++){
            double wx = origin_target[i].x;
            double wy = origin_target[i].y;
            Eigen::Matrix3d Cw;
            Cw <<   1.0, 0.0, -wx,
                    0.0, 1.0, -wy,
                    -wx, -wy, pow(wx,2)+pow(wy,2)-pow(*p_radius,2);

            Eigen::Matrix3d E, E_inv, Qn;
            Eigen::Vector3d rot_vector{rot[0],rot[1],rot[2]};
            
            
            E= LieAlgebra::to_E(se3(rot_vector,Eigen::Vector3d(trans[0],trans[1],trans[2])));
            vector<double> ds = {1, distorsion[0], distorsion[1],distorsion[2],distorsion[3]};
            Point dp(0,0);
            if(mode ==0){
                E_inv=  E.inverse();
                Qn = E_inv.transpose()*Cw*E_inv;
                dp = tracker->ne2dp(Qn,ds);
            }
            else if(mode == 1){
                E_inv=  E.inverse();
                Qn = E_inv.transpose()*Cw*E_inv;
                array<double,5> ellipse_n= tracker->ellipse2array(Qn);
                Point pn(ellipse_n[0],ellipse_n[1]);
                dp = tracker->distort_Point(pn,ds);
            }
            else if(mode == 2){
                Eigen::Vector3d Pw{wx,wy,1};
                Eigen::Vector3d Pn = E*Pw;
                Point pn(Pn[0]/Pn[2],Pn[1]/Pn[2]);
                dp = tracker->distort_Point(pn,ds);
            }
            else{
                throw CalibratorError();
            }

            double u_e = dp.x*fs[0]+dp.y*(*p_skew)+cs[0]; 
            double v_e = dp.y*fs[1]+cs[1];

            double u_o = target[i].x;
            double v_o = target[i].y;

            residual[2*i]= u_o-u_e;
            residual[2*i+1]= v_o-v_e;                
        }
        
        return true;
    }
    private:
        int mode;
        vector<Point> origin_target;
        vector<Point> target;
        MomentumTracker* tracker;
};

class Calibrator{
    public:
        Calibrator(int n_x, int n_y, int n_d,double r, double distance, int max_scene);
        ~Calibrator() = default;

        void inputTarget(vector<cv::Point2f> target);
        void init();
        bool cal_initial_params(Params* inital_params);
        Params calibrate(int mode);
        void printParams(Params p, bool full);
        int get_num_scene();
        void get_extrinsic(string root_dir);
        vector<se3> get_extrinsic();

    private:
        int height, width;
        int num_scene;
        int max_scene;
        int n_x, n_y, n_d;
        double distance;
        double original_r;
        double curr_r;
        double thr_homography;

        vector<se3> Es; // [R1, R2, t] se3: rot , trans
        vector<pair<Eigen::Matrix3d,double>> Hs;

        vector<vector<Point>> targets;
        vector<vector<Point>> ud_targets;
        vector<Point> origin_target;
        array<double, 4> ori_ms; //original_mean_sigma


        double cal_reprojection_error(std::vector<int> sample,Params params, int mode);
        void set_origin_target();
        void set_inital_Es(Params params);
        void normalize_Es();
        array<double, 4> get_mean_sigma(const vector<Point> &target);
        Eigen::VectorXd get_Vij(const Eigen::Matrix3d &H, int i, int j);
        pair<Eigen::Matrix3d,double> get_H(const vector<Point> &target);
        Eigen::VectorXd get_absolute_conic();
        Eigen::MatrixXd normalize_matrix(Eigen::MatrixXd matrix);
        void update_extrinsic(Params intrinsic);

        std::vector<int> get_randomSample(int range, int n);
        Params batch_optimize(std::vector<int> sample, Params initial_params,int phase, int mode);
};




#endif