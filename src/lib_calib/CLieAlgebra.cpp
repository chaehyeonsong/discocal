#include "CLieAlgebra.h"

Eigen::Matrix3d LieAlgebra::skew_symmetric_matrix(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;
    return skew;
}


// Eigen::Vector3d LieAlgebra::to_so3(const Eigen::Matrix3d &R)
// {
//     double trace_R = R.trace();
//     if(trace_R >3 || trace_R <-3) {
//         cout<< R*R.transpose()<<endl;
//         throw LieAlgebraError();
//     }
//     double theta = acos((trace_R - double(1.0)) / double(2.0));

//     if (theta == double(0.))
//         return Eigen::Vector3d::Zero();

//     Eigen::Matrix3d skew_m = (theta / (double(2.) * sin(theta))) * (R - R.transpose());
//     Eigen::Vector3d skew_v(skew_m(2, 1), skew_m(0, 2), skew_m(1, 0));
//     return skew_v;
// }



// Eigen::Vector3d LieAlgebra::normalize_so3(const Eigen::Vector3d &_w)
// {
//     Eigen::Vector3d w = _w;
//     double scale = w.norm()/M_PI;
//     if(scale>1){
//         int share = int(scale);
//         if(share%2 ==0){
//             w = w/scale*(scale-share);
//         }
//         else{
//             w = w/scale*(scale-share-1);
//         }
//     }
//     return w;
// }

// Eigen::Matrix3d LieAlgebra::to_SO3(const Eigen::Vector3d & _w)
// {
//     Eigen::Vector3d w = normalize_so3(_w);
//     // Eigen::Vector3d w = _w;
//     double theta = w.norm();
//     if (theta == double(0))
//         return Eigen::Matrix3d::Identity();

//     Eigen::Matrix3d w_hat = skew_symmetric_matrix(w);
//     Eigen::Matrix3d mat = Eigen::Matrix3d::Identity() + (sin(theta) / theta) * w_hat + ((1.0 - cos(theta)) / (theta * theta)) * (w_hat * w_hat);

//     return mat;
// }
// Eigen::Matrix3d LieAlgebra::to_E(se3 se3){
    
//     Eigen::Matrix3d Rot, E;
//     Rot= LieAlgebra::to_SO3(se3.rot);
//     E.col(0)= Rot.col(0);
//     E.col(1)= Rot.col(1);
//     E.col(2)= se3.trans;

//     return E;
// }

Eigen::Matrix4d LieAlgebra::to_SE3(se3 se3){
    Eigen::Matrix4d T;
    Eigen::Matrix3d Rot = LieAlgebra::to_SO3(se3.rot);
    Eigen::Vector3d trans = se3.trans;
    T<< Rot(0,0) , Rot(0,1) , Rot(0,2) , trans(0),
        Rot(1,0) , Rot(1,1) , Rot(1,2) , trans(1),
        Rot(2,0) , Rot(2,1) , Rot(2,2) , trans(2),
        0        , 0        , 0        , 1.0;
    
    return T;
}

se3 LieAlgebra::to_se3(const Eigen::Matrix4d &T){
    Eigen::Vector3d trans;
    Eigen::Matrix3d rot;
    trans<< T(0,3),T(1,3),T(2,3);
    rot <<T(0,0),T(0,1),T(0,2),
          T(1,0),T(1,1),T(1,2),
          T(2,0),T(2,1),T(2,2);

    return se3(LieAlgebra::to_so3(rot), trans);
}

pair<double, double> LieAlgebra::dist(se3 a,se3 b){
    // d_rot: radian, d_trans: m
    double d_trans = (a.trans-b.trans).norm();
    double costheta, d_rot;
    costheta = ((LieAlgebra::to_SO3(a.rot).transpose() * LieAlgebra::to_SO3(b.rot)).trace()-1)*0.5;
    costheta = min(1.0, max(-1.0,costheta));
    d_rot = acos(costheta);
    return pair<double, double>(d_rot, d_trans);
}

//Todo
// Eigen::Vector4d to_quat(se3 se3){

// }

se3 LieAlgebra::to_se3(const Eigen::Matrix3d &E){
    Eigen::Vector3d r1 = E.col(0);
    Eigen::Vector3d r2 = E.col(1);
    double scale = (r1.norm()+r2.norm())/2;
    Eigen::Vector3d trans = E.col(2)/scale;

    r1 = r1/r1.norm();
    r2 = r2 - r2.dot(r1)*r1;
    r2 = r2/r2.norm();
    Eigen::Vector3d r3 = r1.cross(r2);
    Eigen::Matrix3d R;
    R.col(0)= r1;
    R.col(1)= r2;
    R.col(2)= r3;
    return se3(LieAlgebra::to_so3(R),trans);
}



Eigen::Vector3d LieAlgebra::random_vector(double min, double max){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(min, max);
    double t1 = distribution(gen);
    double t2 = distribution(gen);
    double t3 = distribution(gen);
    Eigen::Vector3d t;
    t << t1, t2, t3;
    return t;
}
Eigen::Matrix3d LieAlgebra::randomE(Eigen::Vector3d standard, double delta){
    Eigen::Vector3d t = LieAlgebra::random_vector(-1.0,1.0);
    t = t/6*delta + standard;
    Eigen::Vector3d  h = LieAlgebra::random_vector(-1.0,1.0);
    h = h/10*delta + Eigen::Vector3d(1,0,0);
    h = h/h.norm()*M_PI;
    Eigen::Matrix3d Rot = to_SO3(h);
    
    Eigen::Matrix3d E;

    E<< Rot(0,0), Rot(0,1), t(0),
        Rot(1,0), Rot(1,1), t(1),
        Rot(2,0), Rot(2,1), t(2);

    return E;
}

se3 LieAlgebra::solveAXXB(const vector<pair<se3, se3>> &ABs){
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    int n = ABs.size();
    Eigen::MatrixXd C(3*n,3);
    Eigen::VectorXd d(3*n);
    Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();
    for(int i=0;i<n;i++){
        Eigen::Vector3d alpha, beta;
        alpha = ABs[i].first.rot;
        beta = ABs[i].second.rot;
        M = M + beta*alpha.transpose();
    }

    Eigen::BDCSVD<Eigen::MatrixXd> svd(M,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d S = svd.singularValues().asDiagonal().toDenseMatrix();
    Eigen::Matrix3d R = V*S.inverse()*V.transpose()*M.transpose();

    C.setZero();
    d.setZero();
    for(int i=0;i<n;i++){
        C.block<3,3>(3*i,0) = I33 - LieAlgebra::to_SO3(ABs[i].first.rot);
        d.block<3,1>(3*i,0) = ABs[i].first.trans - R*ABs[i].second.trans;
    }
    Eigen::Vector3d t = (C.transpose()*C).inverse()*C.transpose()*d;

    se3 x(LieAlgebra::to_so3(R),t);
    return x;
}