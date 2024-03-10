#ifndef __CLieAlgebra_H__
#define __CLieAlgebra_H__


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <random>
#include <iostream>

class LieAlgebraError: public std::exception{
    public:
        const char* what(){
            return "LieAlgebra error";
        }
};
struct se3{
    Eigen::Vector3d rot;
    Eigen::Vector3d trans;
    se3(Eigen::Vector3d _rot, Eigen::Vector3d _trans) : rot(_rot), trans(_trans){};
};

using namespace std;

class LieAlgebra{
    public:
        // transformation between matrix form and screw vector fom
        static Eigen::Matrix3d randomE(Eigen::Vector3d standard, double delta);
        
        static Eigen::Vector3d to_so3(const Eigen::Matrix3d &R);
        static Eigen::Matrix3d to_SO3(const Eigen::Vector3d &v);
        static Eigen::Matrix3d to_E(se3 se3);
        static Eigen::Matrix4d to_SE3(se3 se3);
        static se3 to_se3(const Eigen::Matrix4d &T);
        static se3 to_se3(const Eigen::Matrix3d &E);
        static pair<double, double> dist(se3 a,se3 b);
        static se3 solveAXXB(const vector<pair<se3, se3>> &ABs);



    private:
        static Eigen::Vector3d random_vector(double min, double max);
        static Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v);

};

#endif