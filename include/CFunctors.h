#ifndef __C_FUNCTORS_H__
#define __C_FUNCTORS_H__

#include "utils.h"
#include "CMomentsTracker.h"
#include "CLieAlgebra.h"

using namespace std;


class CalibratorError: public std::exception{
    public:
        const char* what(){
            return "Value exceed the range";
        }
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
        for(int i=0;i<origin_target.size();i++){
            double wx = origin_target[i].x;
            double wy = origin_target[i].y;
            // double radius = 0.05;
            Params params = {fcs[0],fcs[1],fcs[2],fcs[3], fcs[4], distorsion[0],distorsion[1],distorsion[2],distorsion[3]};
            Eigen::Matrix3d E;
            Eigen::Vector3d rot_vector{rot[0],rot[1],rot[2]};  
            rot_vector = LieAlgebra::normalize_so3(rot_vector);

            se3 w2c(rot_vector,Eigen::Vector3d(trans[0],trans[1],trans[2]));
            E= LieAlgebra::to_E(w2c);

            Point p_i = tracker->project(wx, wy, radius, params, E,mode);
            double u_e = p_i.x;
            double v_e = p_i.y;

            double u_o = target[i].x;
            double v_o = target[i].y;

            double c1 = 1;
            double c2 = 0;
            double c3 = 1;

            if(use_weight){
                Eigen::Matrix2d cov;
                cov<<target[i].Kxx , target[i].Kxy , target[i].Kxy , target[i].Kyy;
                Eigen::LLT<Eigen::Matrix2d> lltOfA(cov.inverse()); // compute the Cholesky decomposition of A
                Eigen::Matrix2d L = lltOfA.matrixL(); 
                c1= L(0,0);
                c2= L(1,0);
                c3= L(1,1);
            }

            residual[2*i]= c1*(u_o-u_e)+c2*(v_o-v_e);
            residual[2*i+1]= c3*(v_o-v_e);  
              
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


struct SubCalibrationFunctor {
    SubCalibrationFunctor(const vector<Shape> & origin_target,const vector<Shape>& target, double radius, int n_d, int mode, bool use_weight=false){
        this->origin_target = origin_target;
        this->target = target;
        this->radius= radius;
        this->mode = mode;
        this-> use_weight = use_weight;
        this->tracker = new MomentsTracker(n_d);
    }

    template <typename T>
    bool operator()(const T* const fcs, const T* const distorsion, const T* const rot, const T* const trans, const T* const rot_ext, const T* const trans_ext, T* residual) const {
        for(int i=0;i<origin_target.size();i++){
            double wx = origin_target[i].x;
            double wy = origin_target[i].y;
            // double radius = 0.05;
            Params params = {fcs[0],fcs[1],fcs[2],fcs[3], fcs[4], distorsion[0],distorsion[1],distorsion[2],distorsion[3]};
            Eigen::Vector3d rot_vector{rot[0],rot[1],rot[2]};  
            rot_vector = LieAlgebra::normalize_so3(rot_vector);
            se3 w2c1(rot_vector,Eigen::Vector3d(trans[0],trans[1],trans[2]));

            Eigen::Vector3d rot_vector_ext{rot_ext[0],rot_ext[1],rot_ext[2]};  
            rot_vector_ext = LieAlgebra::normalize_so3(rot_vector_ext);
            se3 c22c1(rot_vector_ext,Eigen::Vector3d(trans_ext[0],trans_ext[1],trans_ext[2]));

            Eigen::Matrix4d w2c2_mat = LieAlgebra::to_SE3(c22c1).inverse() * LieAlgebra::to_SE3(w2c1);
            se3 w2c2 =LieAlgebra::to_se3(w2c2_mat);

            Eigen::Matrix3d E= LieAlgebra::to_E(w2c2);
            // E<< 0.98165199810406873, 
            Point p_i = tracker->project(wx, wy, radius, params, E,mode);
            double u_e = p_i.x;
            double v_e = p_i.y;

            double u_o = target[i].x;
            double v_o = target[i].y;

            double c1 = 1;
            double c2 = 0;
            double c3 = 1;

            if(use_weight){
                Eigen::Matrix2d cov;
                cov<<target[i].Kxx , target[i].Kxy , target[i].Kxy , target[i].Kyy;
                Eigen::LLT<Eigen::Matrix2d> lltOfA(cov.inverse()); // compute the Cholesky decomposition of A
                Eigen::Matrix2d L = lltOfA.matrixL(); 
                c1= L(0,0);
                c2= L(1,0);
                c3= L(1,1);
            }

            residual[2*i]= c1*(u_o-u_e)+c2*(v_o-v_e);
            residual[2*i+1]= c3*(v_o-v_e);  
              
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