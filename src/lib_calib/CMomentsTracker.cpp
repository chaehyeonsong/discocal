#include "CMomentsTracker.h"

MomentsTracker::MomentsTracker(int dim)
{
    this-> max_dim = dim;
    this->max_n = max_dim*3;
    this->comb_max = max_n*2+1;
    init();
}

MomentsTracker::MomentsTracker(){
    this-> max_dim = 4;
    this->max_n = max_dim*3;
    this->comb_max = max_n*2+1;
    init();
}
MomentsTracker::~MomentsTracker()
{
}

void MomentsTracker::init(){
    comb_buffer.reserve(comb_max+1);
    for(int i=0; i<comb_max+1;i++){
        vector<int> temp(i+1,0);
        comb_buffer.push_back(temp);
        comb_buffer[i][0]=1;
        comb_buffer[i][i]=1;
    }

    for(int i=1;i<comb_max+1;i++){
        for(int j=1;j<i;j++){
            comb_buffer[i][j]=comb_buffer[i-1][j-1]+comb_buffer[i-1][j];
        }
    }

    // init moment //
    moment_buffer.reserve(max_n+1);
    for(int i=0; i<max_n+1;i++){
        vector<double> temp(max_n-i+1,0);
        moment_buffer.push_back(temp);
        for(int j=0; j< max_n-i+1;j++){
            moment_buffer[i][j]= _M_2i2j(i,j);
        }
    }

}


double MomentsTracker::_M_2i2j(int i, int j){
    // a, b == 1
    double den= nCr<double>(2*(i+j),i+j)*nCr<double>(i+j,i);
    double num= nCr<double>(2*(i+j),2*i)*(i+j+1)*pow(2,2*(i+j));
    // cout<<den<<num<<endl;
    return den/num;
    
}
double MomentsTracker::M_2i2j(int i, int j){
    if(i> max_n || j>max_n) throw MomentsTrackerError();
    else return moment_buffer[i][j];
}

Point MomentsTracker::ne2dp(Eigen::Matrix3d Q, vector<double> ds){
    /*
    input: ellips in normal plane and distortion parameter
    output: centor point of region of distorted ellipse
    */

    return ne2dp<double>(Q, ds);
}

Point MomentsTracker::distort_Point(Point pn, vector<double> ds){
    return distort_Point<double>(pn, ds);
}

Point MomentsTracker::wc2dp_Numerical(Eigen::Matrix3d Cw, Eigen::Matrix3d H, vector<double> ds, int total_iter){
    return wc2dp_Numerical<double>(Cw, H, ds, total_iter);
}


Point MomentsTracker::ne2dp_Numerical(Eigen::Matrix3d Q, vector<double> ds){
    return ne2dp_Numerical<double>(Q, ds);
}


Point MomentsTracker::project(double wx, double wy, double r, Params intrinsic, Eigen::Matrix3d E, int mode) {
    return project<double>(wx, wy, r, intrinsic, E, mode);

}