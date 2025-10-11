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


class Calibrator{
    public:
        Calibrator(int n_x, int n_y, int n_d,double r, double distance, int max_scene, string result_path);
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

struct CalibrationFunctor {
    CalibrationFunctor(const vector<Shape> & origin_target,const vector<Shape>& target, double radius, int n_d, int mode, bool use_weight=false){
        this->origin_target = origin_target;
        this->target = target;
        this->radius= radius;
        this->mode = mode;
        this-> use_weight = use_weight;
        this->tracker = new MomentsTracker(n_d);
    }

    template <typename T>
    bool operator()(const T* const fcs, const T* const distorsion, const T* const rot, const T* const trans,T* residual) const {
        for(size_t i=0; i<origin_target.size(); i++) {
            T wx = static_cast<T>(origin_target[i].x);
            T wy = static_cast<T>(origin_target[i].y);
            
            Params_T<T> params = {
                fcs[0], fcs[1], fcs[2], fcs[3], fcs[4], 
                distorsion[0], distorsion[1], distorsion[2], distorsion[3]
            };
            
            Eigen::Matrix<T, 3, 3> E;
            Eigen::Matrix<T, 3, 1> rot_vector;
            rot_vector << rot[0], rot[1], rot[2];
            
            rot_vector = LieAlgebra::normalize_so3(rot_vector);

            Eigen::Matrix<T, 3, 1> trans_vector;
            trans_vector << trans[0], trans[1], trans[2];
            
            E = LieAlgebra::to_E(se3_T<T>(rot_vector, trans_vector));

            Point_T<T> p_i = tracker->project<T>(wx, wy, static_cast<T>(radius), params, E, mode);
            T u_e = p_i.x;
            T v_e = p_i.y;

            T u_o = static_cast<T>(target[i].x);
            T v_o = static_cast<T>(target[i].y);

            T c1 = T(1.0);
            T c2 = T(0.0);
            T c3 = T(1.0);

            if(use_weight){
                Eigen::Matrix<T, 2, 2> cov;
                cov << static_cast<T>(target[i].Kxx), static_cast<T>(target[i].Kxy), 
                    static_cast<T>(target[i].Kxy), static_cast<T>(target[i].Kyy);

                Eigen::LLT<Eigen::Matrix<T, 2, 2>> lltOfA(cov.inverse());
                Eigen::Matrix<T, 2, 2> L = lltOfA.matrixL(); 
                c1= L(0,0);
                c2= L(1,0);
                c3= L(1,1);
            }

            residual[2*i] = c1*(u_o-u_e)+c2*(v_o-v_e);
            residual[2*i+1] = c3*(v_o-v_e);
        }
        
        return true;

              
    }
    private:
        int mode;
        // double distance_ratio,radius_ratio;
        bool use_weight;
        double radius;
        vector<Shape> origin_target;
        vector<Shape> target;
        MomentsTracker* tracker;
        // bool normalize;
};

#endif