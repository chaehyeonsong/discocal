#include "CImagehandler.h"


Imagehandler::Imagehandler(int width, int height,Params params, int n_d)
{
    this->width = width;
    this->height = height;
    this->params = params;
    this->n_d = n_d;
    init();
    
}

Imagehandler::~Imagehandler()
{
}

void Imagehandler::init(){
    cv::Size imageSize = cv::Size(width,height);

    //************** compatable with opencv ************//
    // cv::Mat camera_matrix= cv::Mat::eye(3, 3, CV_64FC1);
    // cv::Mat distcoeff = cv::Mat::zeros(1, 5, CV_64FC1);
    // camera_matrix=(cv::Mat1d(3, 3) <<  params.fx, params.skew, params.cx, 0., params.fy, params.cy, 0., 0.,1. );
    // if(n_d==1){
    //     distcoeff=(cv::Mat1d(1, 5) << params.d[0],0., 0., 0.,0.);
    // }
    // else if (n_d ==2){
    //     distcoeff=(cv::Mat1d(1, 5) << params.d[0],params.d[1], 0., 0.,0.);
    // }
    // else{
    //     distcoeff=(cv::Mat1d(1, 5) << params.d[0],params.d[1],0., 0., params.d[2]);
    // }
    // cv::initUndistortRectifyMap(camera_matrix, distcoeff, cv::Mat(), camera_matrix, imageSize, CV_32FC1, mapx, mapy);
    //************** compatable with opencv ************//

    Eigen::ArrayXXf x_array = Eigen::RowVectorXf::LinSpaced(width, 0, width-1).replicate(height,1).array();
    Eigen::ArrayXXf y_array = Eigen::VectorXf::LinSpaced(height, 0, height-1).replicate(1,width).array();

    y_array = (y_array-params.cy)/params.fy;
    x_array = (x_array-params.cx-params.skew*y_array)/params.fx;
    Eigen::ArrayXXf s_matrix_1 = x_array*x_array+y_array*y_array;
    Eigen::ArrayXXf s_matrix_2 = s_matrix_1*s_matrix_1;
    Eigen::ArrayXXf s_matrix_3 = s_matrix_2*s_matrix_1;
    Eigen::ArrayXXf s_matrix_4 = s_matrix_3*s_matrix_1;
    Eigen::ArrayXXf k_matrix = 1+s_matrix_1*params.d[0]+s_matrix_2*params.d[1]+s_matrix_3*params.d[2]+s_matrix_4*params.d[3];
    x_array = k_matrix*x_array;
    y_array = k_matrix*y_array;
    x_array = params.fx*x_array+params.skew*y_array+params.cx;
    y_array = params.fy*y_array+params.cy;

    Eigen::MatrixXf x_matrix(width,height);
    Eigen::MatrixXf y_matrix(width,height);
    x_matrix = x_array.matrix();
    y_matrix = y_array.matrix();
    cv::eigen2cv(x_matrix,mapx);
    cv::eigen2cv(y_matrix,mapy);
}

cv::Mat Imagehandler::undistort(cv::Mat image){
    cv::Mat undist_image;
    cv::remap(image,undist_image, mapx, mapy, cv::INTER_LINEAR);
    return undist_image;
}