#ifndef __CLieAlgebra_H__
#define __CLieAlgebra_H__


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include <math.h>
#include <random>
#include <iostream>

class LieAlgebraError: public std::exception{
    public:
        const char* what(){
            return "LieAlgebra error";
        }
};
// struct so3{
//     Eigen::Vector3d w;
// };
// struct se3{
//     Eigen::Vector3d rot;
//     Eigen::Vector3d trans;
//     se3(Eigen::Vector3d _rot, Eigen::Vector3d _trans) : rot(_rot), trans(_trans){};
//     se3() : rot(Eigen::Vector3d::Zero()), trans(Eigen::Vector3d::Zero()){};
//     std::string to_string() const {
//         std::string message = std::to_string(rot(0))+"\t"+std::to_string(rot(1))+"\t"+std::to_string(rot(2))+
//         "\t"+std::to_string(trans(0))+"\t"+std::to_string(trans(1))+"\t"+std::to_string(trans(2));
//         return message;
//     }
//     se3 from_string(const std::string& str) {
//         se3 result;
//         std::stringstream ss(str);
//         std::string segment;
//         std::vector<double> values;

//         for (int i=0; i<6; i++) {
//             if (std::getline(ss, segment, '\t')) {
//                 try {
//                     values.push_back(std::stod(segment));
//                 }
//                 catch (const std::invalid_argument& e) {
//                     throw std::runtime_error("from_string: Invalid number format in segment '" + segment + "': " + e.what());
//                 }
//                 catch (const std::out_of_range& e) {
//                     throw std::runtime_error("from_string: Number out of range in segment '" + segment + "': " + e.what());
//                 }
//             }
//             else {
//                 throw std::runtime_error("from_string: Not enough segments in string. Expected 6, got " + std::to_string(i));
//             }
//         }

//         if (values.size() != 6) {
//             throw std::runtime_error("from_string: Incorrect number of values parsed. Expected 6, got " + std::to_string(values.size()));
//         }

//         result.rot(0) = values[0];
//         result.rot(1) = values[1];
//         result.rot(2) = values[2];
//         result.trans(0) = values[3];
//         result.trans(1) = values[4];
//         result.trans(2) = values[5];

//         return result;
//     }
// };

template <typename T>
struct so3_T{
    Eigen::Matrix<T, 3, 1> w;
};

template <typename T>
struct se3_T{
    Eigen::Matrix<T, 3, 1> rot;
    Eigen::Matrix<T, 3, 1> trans;
    se3_T(const Eigen::Matrix<T, 3, 1>& _rot, const Eigen::Matrix<T, 3, 1>& _trans) : rot(_rot), trans(_trans){};
    se3_T() : rot(Eigen::Matrix<T, 3, 1>::Zero()), trans(Eigen::Matrix<T, 3, 1>::Zero()){};
    std::string to_string() const {
        std::string message = std::to_string(rot(0))+"\t"+std::to_string(rot(1))+"\t"+std::to_string(rot(2))+
        "\t"+std::to_string(trans(0))+"\t"+std::to_string(trans(1))+"\t"+std::to_string(trans(2));
        return message;
    }
    se3_T from_string(const std::string& str) {
        se3_T result;
        std::stringstream ss(str);
        std::string segment;
        std::vector<double> values;

        for (int i=0; i<6; i++) {
            if (std::getline(ss, segment, '\t')) {
                try {
                    values.push_back(std::stod(segment));
                }
                catch (const std::invalid_argument& e) {
                    throw std::runtime_error("from_string: Invalid number format in segment '" + segment + "': " + e.what());
                }
                catch (const std::out_of_range& e) {
                    throw std::runtime_error("from_string: Number out of range in segment '" + segment + "': " + e.what());
                }
            }
            else {
                throw std::runtime_error("from_string: Not enough segments in string. Expected 6, got " + std::to_string(i));
            }
        }

        if (values.size() != 6) {
            throw std::runtime_error("from_string: Incorrect number of values parsed. Expected 6, got " + std::to_string(values.size()));
        }

        result.rot(0) = values[0];
        result.rot(1) = values[1];
        result.rot(2) = values[2];
        result.trans(0) = values[3];
        result.trans(1) = values[4];
        result.trans(2) = values[5];

        return result;
    }
};

typedef so3_T<double> so3;

typedef se3_T<double> se3;



using namespace std;

class LieAlgebra{
    public:
        // transformation between matrix form and screw vector fom
        static Eigen::Matrix3d randomE(Eigen::Vector3d standard, double delta);
        // static Eigen::Vector3d to_so3(const Eigen::Matrix3d &R);
        // static Eigen::Vector3d normalize_so3(const Eigen::Vector3d &_w);
        // static Eigen::Matrix3d to_SO3(const Eigen::Vector3d &_w);
        // static Eigen::Matrix3d to_E(se3 se3);
        static Eigen::Matrix4d to_SE3(se3 se3);
        static se3 to_se3(const Eigen::Matrix4d &T);
        static se3 to_se3(const Eigen::Matrix3d &E);
        static pair<double, double> dist(se3 a,se3 b);
        static se3 solveAXXB(const vector<pair<se3, se3>> &ABs);


        //template ftn
        template<typename T>
        static Eigen::Matrix<T, 3, 1> to_so3(const Eigen::Matrix<T, 3, 3> &R) {
            T trace_R = R.trace();
            if(trace_R > T(3.) || trace_R < T(-3.)) {
                cout<< R*R.transpose()<<endl;
                throw LieAlgebraError();
            }

            T theta = ceres::acos((trace_R - T(1.0)) / T(2.0));

            if (ceres::abs(theta) < T(1e-12)) return Eigen::Matrix<T, 3, 1>::Zero();

            Eigen::Matrix<T, 3, 3> skew_m = (theta / (T(2.) * ceres::sin(theta))) * (R - R.transpose()); // ceres::sin 사용
            Eigen::Matrix<T, 3, 1> skew_v(skew_m(2, 1), skew_m(0, 2), skew_m(1, 0));
            // std::cout << skew_v << std::endl;
            return skew_v;
        }

        template<typename T>
        static Eigen::Matrix<T, 3, 1> normalize_so3(const Eigen::Matrix<T, 3, 1> &_w) {
            Eigen::Matrix<T, 3, 1> w = _w;
            T norm_w = w.norm();
            T scale = norm_w / T(M_PI);

            if (scale > T(1)) {
                int share;

                if constexpr (std::is_same_v<T, double>) {
                    share = static_cast<int>(ceres::floor(scale));
                }
                else {
                    share = static_cast<int>(ceres::floor(scale).a);
                }

                if (share % 2 == 0) {
                    w = w / scale * (scale - T(static_cast<double>(share)));
                }
                else {
                    w = w / scale * (scale - T(static_cast<double>(share)) - T(1));
                }
            }
            return w;
        }

        template<typename T>
        static Eigen::Matrix<T, 3, 3> to_SO3(const Eigen::Matrix<T, 3, 1> & _w) {
            Eigen::Matrix<T, 3, 1> w = normalize_so3(_w);
            T theta = w.norm();

            if (ceres::abs(theta) < T(1e-12))
                return Eigen::Matrix<T, 3, 3>::Identity();

            Eigen::Matrix<T, 3, 3> w_hat = skew_symmetric_matrix(w);
            Eigen::Matrix<T, 3, 3> mat = Eigen::Matrix<T, 3, 3>::Identity() + (ceres::sin(theta) / theta) * w_hat + ((T(1.0) - ceres::cos(theta)) / (theta * theta)) * (w_hat * w_hat);

            return mat;
        }


        template<typename T>
        static Eigen::Matrix<T, 3, 3> to_E(se3_T<T> se3_val) {
            Eigen::Matrix<T, 3, 3> Rot, E;
            Rot = LieAlgebra::to_SO3(se3_val.rot);
            E.col(0)= Rot.col(0);
            E.col(1)= Rot.col(1);
            E.col(2)= se3_val.trans;

            return E;
        }



    private:
        static Eigen::Vector3d random_vector(double min, double max);
        static Eigen::Matrix3d skew_symmetric_matrix(const Eigen::Vector3d &v);

        //template ftn
        template<typename T>
        static Eigen::Matrix<T, 3, 3> skew_symmetric_matrix(const Eigen::Matrix<T, 3, 1> &v) {
            Eigen::Matrix<T, 3, 3> skew;
            skew << T(0.), -v(2), v(1),
                    v(2), T(0.), -v(0),
                    -v(1), v(0), T(0.);
            return skew;
        }

};

#endif