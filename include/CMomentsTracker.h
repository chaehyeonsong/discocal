#ifndef __CMOMENTSTRACKER_H__
#define __CMOMENTSTRACKER_H__

#include <vector>
#include <array>
#include <iostream>
#include "math.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "utils.h"
using namespace std;


// class MomentsTrackerError: public std::exception{
//     public:
//         const char* what(){
//             return "Value exceed the range";
//         }
// };

class MomentsTrackerError : public std::exception {
private:
    std::string message;

public:
    explicit MomentsTrackerError(const std::string& msg)
        : message(msg) {}
    const char* what() const noexcept override {
        return message.c_str();
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
    // double C0n(int n, vector<double> ds);
    // double C1n(int n, vector<double> ds);
    // double nCr(int i, int j);
    double I_2i2j(int i, int j);
    double M_2i2j(int i, int j);
    // double M_2i2j(int i, int j, double a, double b);
    // array<double ,3> intSn(int n,double a, double b, double tx, double ty,double alpha);


    //template ftn
    template<typename T>
    T C0n(int n, const std::vector<T>& ds) {
        T result = T(0);
        int n_d = max_dim;
        for (int i = std::max(0, n - n_d); i < std::min(n, n_d) + 1; i++) {
            result += T(2 * i + 1) * ds[i] * ds[n - i];
        }
        return result;
    }

    template<typename T>
    T C1n(int n, const std::vector<T>& ds) {
        T result = T(0);
        int n_d = max_dim;
        for (int i = std::max(0, n - n_d * 2); i < std::min(n, n_d) + 1; i++) {
            T temp = T(0);
            for (int j = std::max(0, n - i - n_d); j < std::min(n - i, n_d) + 1; j++) {
                temp += ds[j] * ds[n - i - j];
            }
            result += T(2 * i + 1) * ds[i] * temp;
        }
        return result;
    }

    template<typename T>
    T nCr(int n, int r) {
        if (n > comb_max || n < r) throw MomentsTrackerError("nCr error!!");
        return T(static_cast<double>(comb_buffer[n][r]));  
    }

    template<typename T>
    T M_2i2j(int i, int j, T a, T b) {
        T base_M_val = T(MomentsTracker::M_2i2j(i, j));
        return base_M_val * ceres::pow(a, T(2 * i)) * ceres::pow(b, T(2 * j)); 
    }

    template<typename T>
    array<T, 3> intSn(int n, T a, T b, T tx, T ty, T alpha) {
        T tx_s = ceres::cos(alpha) * tx + ceres::sin(alpha) * ty;
        T ty_s = -ceres::sin(alpha) * tx + ceres::cos(alpha) * ty;
        T s0 = T(0), sx = T(0), sy = T(0);
        std::array<T, 3> results;

        for (int i = 0; i < n + 1; i++) {
            for (int j = 0; j < n - i + 1; j++) {
                T M = M_2i2j(i, j, a, b);
                T m_0 = T(0);
                T m_x = T(0);
                T m_y = T(0);
                for (int k = i; k < n - j + 1; k++) {
                    m_0 += nCr<double>(n, k) * nCr<double>(2 * k, 2 * i) * nCr<double>(2 * (n - k), 2 * j) *
                        ceres::pow(tx_s, T(2 * (k - i))) * ceres::pow(ty_s, T(2 * (n - j - k)));
                    m_x += nCr<double>(n, k) * nCr<double>(2 * k + 1, 2 * i) * nCr<double>(2 * (n - k), 2 * j) *
                        ceres::pow(tx_s, T(2 * (k - i) + 1)) * ceres::pow(ty_s, T(2 * (n - j - k)));
                    m_y += nCr<double>(n, k) * nCr<double>(2 * k, 2 * i) * nCr<double>(2 * (n - k) + 1, 2 * j) *
                        ceres::pow(tx_s, T(2 * (k - i))) * ceres::pow(ty_s, T(2 * (n - j - k) + 1));
                }
                s0 += m_0 * M;
                sx += m_x * M;
                sy += m_y * M;
            }
        }
        results[0] = s0;
        results[1] = ceres::cos(alpha) * sx - ceres::sin(alpha) * sy;
        results[2] = ceres::sin(alpha) * sx + ceres::cos(alpha) * sy;
        return results;
    }

    

public:
    MomentsTracker(int dim);
    MomentsTracker();
    ~MomentsTracker();


    Point ne2dp(Eigen::Matrix3d Q, vector<double> ds);

    Point wc2dp_Numerical(Eigen::Matrix3d Cw,Eigen::Matrix3d H, vector<double> ds,int total_iter=1600);
    Point ne2dp_Numerical(Eigen::Matrix3d Q, vector<double> ds);
    
    Point project(double wx, double wy, double r,Params intrinsic, Eigen::Matrix3d E, int mode);

    Point distort_Point(Point pn, vector<double> ds);


    //template ftn
    template<typename T>
    Point_T<T> ne2dp(Eigen::Matrix<T, 3, 3> Q, const std::vector<T>& ds) {
        /*
        input: ellips in normal plane and distortion parameter
        output: centor point of region of distorted ellipse
        */

        std::array<T, 5> ellipse = ellipse2array(Q);

        T a, b, tx, ty, alpha;
        tx = ellipse[0];
        ty = ellipse[1];
        a = ellipse[2];
        b = ellipse[3];
        alpha = ellipse[4];
        T A_d = T(0);
        T x_d = T(0);
        T y_d = T(0);

        for (int n = 0; n < 3 * max_dim + 1; n++) {
            T c_0n = C0n(n, ds);
            T c_1n = C1n(n, ds); 
            std::array<T, 3> all_Sn = intSn(n, a, b, tx, ty, alpha);
            A_d += c_0n * all_Sn[0];
            x_d += c_1n * all_Sn[1];
            y_d += c_1n * all_Sn[2];
        }

        x_d = x_d / A_d;
        y_d = y_d / A_d;
        return Point_T<T>(x_d, y_d);
    }

    template<typename T>
    Point_T<T> project(T wx, T wy, T r, Params_T<T> intrinsic, Eigen::Matrix<T, 3, 3> E, int mode=0) {
        Eigen::Matrix<T, 3, 3> Cw;
        Cw <<   T(1.0), T(0.0), -wx,
                T(0.0), T(1.0), -wy,
                -wx, -wy, ceres::pow(wx,2) + ceres::pow(wy,2) - ceres::pow(r,2); // ceres::pow 사용

        std::vector<T> ds = {T(1), intrinsic.d[0], intrinsic.d[1], intrinsic.d[2], intrinsic.d[3]};

        Point_T<T> dp(T(0),T(0));

        if(mode ==0){
            Eigen::Matrix<T, 3, 3> E_inv=  E.inverse();
            Eigen::Matrix<T, 3, 3> Qn = E_inv.transpose()*Cw*E_inv;
            dp = ne2dp(Qn,ds);
        }
        else{
            throw MomentsTrackerError("project error: only mode=0");
        }

        T u_e = dp.x*intrinsic.fx+dp.y*intrinsic.skew+intrinsic.cx; 
        T v_e = dp.y*intrinsic.fy+intrinsic.cy;

        return Point_T<T>(u_e, v_e);
    }

    template<typename T> 
    array<T, 5> ellipse2array(Eigen::Matrix<T, 3, 3> Q) {
        T a = Q(0, 0), b = Q(0, 1), c = Q(1, 1), d = Q(0, 2), e = Q(1, 2), f = Q(2, 2);
        T x, y, m0, m1, v0, v1;
        T det_Q = Q.determinant(); 
        T det_Q33 = a * c - b * b; 
        T s = a + c;
        x = (-c * d + b * e) / det_Q33;
        y = (b * d - a * e) / det_Q33;

        T k = -det_Q / det_Q33;
        T root = ceres::sqrt(ceres::pow(a - c, T(2)) + T(4) * ceres::pow(b, T(2)));
        T lamda0 = (s - root) / T(2);
        T lamda1 = (s + root) / T(2);
        m0 = ceres::sqrt(k / lamda0);
        m1 = ceres::sqrt(k / lamda1);
        
        if (ceres::abs(b) < T(1e-12)) {
            v0 = T(0);
            v1 = T(1);
        } else {
            v0 = ((c - a) + root) / T(2);
            v1 = -b;
        }
        T alpha = ceres::atan2(v1, v0);

        array<T, 5> result = {x, y, m0, m1, alpha};
        return result;
    }




};

















#endif