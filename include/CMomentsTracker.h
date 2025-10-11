#ifndef __CMOMENTSTRACKER_H__
#define __CMOMENTSTRACKER_H__

#include <vector>
#include <array>
#include <iostream>
#include "math.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
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
    double I_2i2j(int i, int j);
    double M_2i2j(int i, int j);


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
        if (n > comb_max || n < r) throw MomentsTrackerError();
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
    Point_T<T> wc2dp_Numerical(Eigen::Matrix<T, 3, 3> Cw, Eigen::Matrix<T, 3, 3> H, std::vector<T> ds, int total_iter = 1600) {
        int iter = (int)(sqrt(total_iter));
        T A_d = T(0);
        T x_d = T(0);
        T y_d = T(0);

        T t_x = -Cw(0, 2);
        T t_y = -Cw(1, 2);
        T R = ceres::sqrt(-Cw(2, 2) + ceres::pow(t_x, 2) + ceres::pow(t_y, 2));
        T r_step = T(1.0) / T(iter);
        T t_step = T(2 * M_PI) / T(iter);

        for (int i = 0; i < iter + 1; i++) {
            for (int j = 0; j < iter + 1; j++) {
                T r = r_step * (T(i) + T(0.5));
                T t = t_step * (T(j) + T(0.5));

                T x_w = t_x + R * r * ceres::cos(t);
                T y_w = t_y + R * r * ceres::sin(t);
                T f = H(0, 0) * x_w + H(0, 1) * y_w + H(0, 2);
                T g = H(1, 0) * x_w + H(1, 1) * y_w + H(1, 2);
                T h = H(2, 0) * x_w + H(2, 1) * y_w + H(2, 2);
                T x_n = f / h;
                T y_n = g / h;

                T H02 = H(1, 0) * H(2, 1) - H(1, 1) * H(2, 0);
                T H12 = H(0, 0) * H(2, 1) - H(0, 1) * H(2, 0);
                T H22 = H(0, 0) * H(1, 1) - H(0, 1) * H(1, 0);

                T Jwrt = R * R * r;
                T Jnw = (H02 * f / h - H12 * g / h + H22) / ceres::pow(h, 2);

                T s_n = ceres::pow(x_n, 2) + ceres::pow(y_n, 2);
                T tmp1 = T(0);
                T tmp2 = T(0);
                for (int k = 0; k < max_dim + 1; k++) {
                    tmp1 += ds[k] * ceres::pow(s_n, k);
                    tmp2 += (T(2 * k + 1)) * ds[k] * ceres::pow(s_n, k);
                }
                T Jdn = tmp1 * tmp2;

                T x_d_t = tmp1 * x_n;
                T y_d_t = tmp1 * y_n;

                T J = Jdn * Jnw * Jwrt;

                A_d += J * r_step * t_step;
                x_d += x_d_t * J * r_step * t_step;
                y_d += y_d_t * J * r_step * t_step;
            }
        }

        x_d = x_d / A_d;
        y_d = y_d / A_d;
        return Point_T<T>(x_d, y_d);
    }

    template<typename T>
    Point_T<T> ne2dp_Numerical(Eigen::Matrix<T, 3, 3> Q, std::vector<T> ds) {
        int total_iter = 100;
        int iter = (int)(sqrt(total_iter));
        array<T, 5> ellipse = ellipse2array(Q);

        T a, b, tx, ty, alpha;
        tx = ellipse[0];
        ty = ellipse[1];
        a = ellipse[2];
        b = ellipse[3];
        alpha = ellipse[4];
        T A_d = T(0);
        T x_d = T(0);
        T y_d = T(0);

        T r_step = T(1.0) / T(iter);
        T t_step = T(2 * M_PI) / T(iter);

        for (int i = 0; i < iter + 1; i++) {
            for (int j = 0; j < iter + 1; j++) {
                int scale_i = 2;
                int scale_j = 2;
                if (i == 0 || i == iter) scale_i = 1;
                if (j == 0 || j == iter) scale_j = 1;
                T r = r_step * T(i);
                T t = t_step * T(j);

                T x_n = tx + a * r * ceres::cos(alpha) * ceres::cos(t) - b * r * ceres::sin(alpha) * ceres::sin(t);
                T y_n = ty + a * r * ceres::sin(alpha) * ceres::cos(t) + b * r * ceres::cos(alpha) * ceres::sin(t);
                T s_n = ceres::pow(x_n, 2) + ceres::pow(y_n, 2);
                T tmp1 = T(0);
                T tmp2 = T(0);
                for (int k = 0; k < max_dim + 1; k++) {
                    tmp1 += ds[k] * ceres::pow(s_n, k);
                    tmp2 += (T(2 * k + 1)) * ds[k] * ceres::pow(s_n, k);
                }
                T Jdn = tmp1 * tmp2;
                T Jnrt = a * b * r;

                T x_d_t = tmp1 * x_n;
                T y_d_t = tmp1 * y_n;

                A_d += Jdn * Jnrt * r_step * t_step * T(scale_i * scale_j) / T(4);
                x_d += x_d_t * Jdn * Jnrt * r_step * t_step * T(scale_i * scale_j) / T(4);
                y_d += y_d_t * Jdn * Jnrt * r_step * t_step * T(scale_i * scale_j) / T(4);
            }
        }
        x_d = x_d / A_d;
        y_d = y_d / A_d;
        return Point_T<T>(x_d, y_d);
    }

    template<typename T>
    Point_T<T> distort_Point(Point_T<T> pn, std::vector<T> ds) {
        T x, s, y, k;
        x = pn.x;
        y = pn.y;
        s = x * x + y * y;
        k = T(0);
        for (int i = 0; i < max_dim + 1; i++) {
            k += ds[i] * ceres::pow(s, i);
        }
        return Point_T<T>(k * x, k * y);
    }

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
                -wx, -wy, ceres::pow(wx,2) + ceres::pow(wy,2) - ceres::pow(r,2); // ceres::pow

        std::vector<T> ds = {T(1), intrinsic.d[0], intrinsic.d[1], intrinsic.d[2], intrinsic.d[3]};

        Point_T<T> dp(T(0),T(0));
        if(mode == 0){
            Eigen::Matrix<T, 3, 3> E_inv=  E.inverse();
            Eigen::Matrix<T, 3, 3> Qn = E_inv.transpose()*Cw*E_inv;
            dp = ne2dp(Qn,ds);
        }
        else if(mode == 1){
            Eigen::Matrix<T, 3, 3> E_inv=  E.inverse();
            Eigen::Matrix<T, 3, 3> Qn = E_inv.transpose()*Cw*E_inv;
            array<T,5> ellipse_n= ellipse2array(Qn);
            Point_T<T> pn(ellipse_n[0],ellipse_n[1]);
            dp = distort_Point(pn,ds);
        }
        else if(mode == 2){
            Eigen::Matrix<T, 3, 1> Pw;
            Pw << wx, wy, T(1.0);
            Eigen::Matrix<T, 3, 1> Pn = E*Pw;
            Point_T<T> pn(Pn[0]/Pn[2],Pn[1]/Pn[2]);
            dp = distort_Point(pn,ds);
        }
        else if(mode == 3){
            Eigen::Matrix<T, 3, 3> E_inv=  E.inverse();
            Eigen::Matrix<T, 3, 3> Qn = E_inv.transpose()*Cw*E_inv;
            dp = ne2dp_Numerical(Qn,ds);
        }
        else if(mode == 4){
            dp = wc2dp_Numerical(Cw,E,ds);
        }
        else{
            throw MomentsTrackerError();
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