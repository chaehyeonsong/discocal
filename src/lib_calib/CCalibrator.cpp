#include "CCalibrator.h"




Calibrator::Calibrator(int n_x, int n_y,int n_d, double r,double distance, int max_scene){
    this->n_x = n_x;
    this->n_y = n_y;
    this->n_d = n_d;
    this->max_scene = max_scene;
    this->num_scene = 0;
    this->thr_homography = 0;

    this->original_r = r;
    this->distance = distance;

    origin_target.reserve(n_x*n_y);
    targets.reserve(max_scene);
    ud_targets.reserve(max_scene);
    Hs.reserve(max_scene);
    Es.reserve(max_scene);

    set_origin_target();
}
int Calibrator::get_num_scene(){
    return this->num_scene;
}

void Calibrator::init(){

    Es.clear();
    curr_r= original_r;

}

void Calibrator::set_origin_target(){

    for(int j=0;j<n_y;j++){
        for(int i=0;i<n_x;i++){
            double cx,cy;
            cx = distance*(i+1);
            cy = distance*(n_y-j);
            origin_target.push_back(Point(cx,cy));
        }
    }

    ori_ms = get_mean_sigma(origin_target);
}

array<double, 4> Calibrator::get_mean_sigma(const vector<Point> &target){
    double m_x{0}, m_y{0}, s_x{0},s_y{0};
    int n = target.size();
    for(int i=0; i<n;i++){
        m_x+= target[i].x;
        m_y+= target[i].y;
        s_x+= pow(target[i].x,2);
        s_y+= pow(target[i].y,2);
    }
    m_x = m_x/n;
    m_y = m_y/n;
    s_x = sqrt(s_x/n - pow(m_x,2));
    s_y = sqrt(s_y/n - pow(m_y,2));
    array<double, 4> result{m_x, m_y, s_x, s_y};
    return result;
}

pair<Eigen::Matrix3d,double> Calibrator::get_H(const vector<Point> &target){
    
    int n_points = target.size();

    Eigen::MatrixXd X(2*n_points,9);
    X.setZero();

    array<double,4> ms = get_mean_sigma(target);
    double m_u{ms[0]}, m_v{ms[1]}, s_u{ms[2]}, s_v{ms[3]};

    double m_x{ori_ms[0]}, m_y{ori_ms[1]}, s_x{ori_ms[2]}, s_y{ori_ms[3]};

    for (int i=0;i<n_points;i++){
        double x,y,u,v;

        u = (target[i].x-m_u)/s_u;
        v = (target[i].y-m_v)/s_v;
        x = (origin_target[i].x-m_x)/s_x;
        y = (origin_target[i].y-m_y)/s_y;

        X(2*i, 0)=x;
        X(2*i,1) = y;
        X(2*i,2) = 1;
        X(2*i,6) = -u*x;
        X(2*i,7) = -u*y;
        X(2*i,8) = -u;

        X(2*i+1,3)=x;
        X(2*i+1,4) = y;
        X(2*i+1,5) = 1;
        X(2*i+1,6) = -v*x;
        X(2*i+1,7) = -v*y;
        X(2*i+1,8) = -v;

    }
    Eigen::BDCSVD<Eigen::MatrixXd> svd(X,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V = svd.matrixV();

    Eigen::Matrix3d H, Kt, Ko;
    H << V(0,8),V(1,8),V(2,8),
         V(3,8),V(4,8),V(5,8),
         V(6,8),V(7,8),V(8,8);

    Ko << 1/s_x, 0, -m_x/s_x,
            0,  1/s_y, -m_y/s_y,
            0,  0,  1;

    Kt << 1/s_u, 0, -m_u/s_u,
            0,  1/s_v, -m_v/s_v,
            0,  0,  1;

    Eigen::MatrixXd singular_values=svd.singularValues();
    double singular_value_ratio = singular_values(7)/singular_values(8);
    // std::cout<<"singular value ratio: "<<singular_value_ratio<<std::endl;

    H = Kt.inverse() * H * Ko;
    H = normalize_matrix(H);
    pair<Eigen::Matrix3d,double> result(H,singular_value_ratio);
    
    return result;
}


void Calibrator::inputTarget(vector<cv::Point2f> target){
    vector<Point> target2;
    int n = target.size();
    target2.reserve(n);
    for(int i=0;i<n;i++){
        target2.push_back(Point(target[i].x,target[i].y));
    }
    targets.push_back(target2);
    Hs.push_back(get_H(target2));
    // rs.push_back(original_r);
    num_scene++;
    printf("%dth scene stored\n",num_scene);
}


Eigen::MatrixXd Calibrator::normalize_matrix(Eigen::MatrixXd matrix){
    double norm = matrix.norm();
    return matrix/norm;
}

Eigen::VectorXd Calibrator::get_Vij(const Eigen::Matrix3d &H, int i, int j){
    Eigen::Vector3d h_i,h_j;
    h_i= H.col(i);
    h_j= H.col(j);

    Eigen::VectorXd v_ij(6);
    v_ij << h_i(0)*h_j(0), 
            h_i(0)*h_j(1)+h_i(1)*h_j(0),
            h_i(1)*h_j(1),
            h_i(2)*h_j(0)+h_i(0)*h_j(2),
            h_i(2)*h_j(1)+h_i(1)*h_j(2),
            h_i(2)*h_j(2);

    return v_ij;
}

Eigen::VectorXd Calibrator::get_absolute_conic(){
    Eigen::VectorXd b(6);
    Eigen::MatrixXd X(2*this->num_scene,6);
    X.setZero();

    double h_mean=0;
    for(int i=0;i<num_scene;i++){
        h_mean+= Hs.at(i).second;
    }
    h_mean = h_mean/num_scene;
    
    for(int i=0;i<num_scene;i++){
        pair<Eigen::Matrix3d,double> result  = Hs.at(i);
        Eigen::Matrix3d H = result.first;
        // cout<<H<<endl;
        double h_ratio =result.second/h_mean;
        H*= h_ratio;
        if (h_ratio>this->thr_homography){
            Eigen::VectorXd v_01 = get_Vij(H,0,1);
            Eigen::VectorXd v_00 = get_Vij(H,0,0);
            Eigen::VectorXd v_11 = get_Vij(H,1,1);
            X.row(2*i)=v_01;
            X.row(2*i+1) = v_00-v_11;
        }
    }
    Eigen::BDCSVD<Eigen::MatrixXd> svd(X,Eigen::ComputeThinU | Eigen::ComputeThinV);
    b = svd.matrixV().col(5);
    return b;
}





bool Calibrator::cal_initial_params(Params* inital_params){
    if(targets.size()<5){
        printf("Error: algorithm need at leat 5 images");
        return false;
    }
    else{
        Eigen::VectorXd b = get_absolute_conic();
        // cout<<b<<endl;
        // std::cout<<"absolute_conic:\n"<<b<<std::endl;
        double cx,cy,fx,fy,fx2,fy2,skew,lamda;
        cy = (b(1)*b(3)-b(0)*b(4))/(b(0)*b(2)-b(1)*b(1));
        lamda = b(5) - (b(3)*b(3)+cy*(b(1)*b(3)-b(0)*b(4))) /b(0);
        fx2= lamda/b(0);
        fy2= lamda*b(0)/(b(0)*b(2)-b(1)*b(1));
        if(fx2>0 && fy2>0){
            fx = sqrt(fx2);
            fy = sqrt(fy2);
            skew = -b(1)*fx*fx*fy/lamda;
            cx = skew*cy/fx - b(3)*fx*fx/lamda;

            inital_params->fx= fx;
            inital_params->fy= fy;
            inital_params->cx = cx;
            inital_params->cy= cy;
            inital_params->skew = skew;
            return true;
        }
        else{
            printf("Not enough images collected");
            return false;
        }

    }

}

void Calibrator::set_inital_Es(Params params){
    Eigen::Matrix3d K, E, R;
    
    K << params.fx, params.skew , params.cx,
        0,  params.fy,  params.cy,
        0,  0,  1;

    if(K.determinant()==0){
        printf("intrinsic matrix should be invertable\n");
        Eigen::Vector3d rot{M_PI,0,0}, trans{0,0,1.0};
        for(int i=0; i<num_scene;i++){
            Es.push_back(se3(rot, trans));
        }
    }
    else{
        Eigen::Vector3d rot, trans, R1, R2, R3;
        for(int i=0; i<num_scene;i++){
            E = K.inverse()* Hs.at(i).first;
            Es.push_back(LieAlgebra::to_se3(E));
        }
    }
    
}

std::vector<int> Calibrator::get_randomSample(int range, int n){
    std::vector<bool> data(range);
    fill(data.begin(),data.end(), true);

    std::vector<int> sampledData;
    srand((unsigned int)time(NULL));
    while(n>0){
        int num =rand()%range;
        if(data[num]){
            sampledData.push_back(num);
            data[num]=false;
            n--;
        }
    }
    return sampledData;
}

Params Calibrator::batch_optimize(std::vector<int> sample, Params initial_params,bool fix_radus, int mode){
    /*
    phase: set const
     0: default
     1: optimize radius
    */  
    curr_r =original_r;
    double fs[]={initial_params.fx, initial_params.fy};
    double cs[]={initial_params.cx, initial_params.cy};
    double skew = initial_params.skew;
    double distorsion[4]={initial_params.d[0],initial_params.d[1],initial_params.d[2],initial_params.d[3]};

    Problem problem; 

    for(int i=0;i<sample.size();i++){
        int index =sample.at(i);
        int num_residual= targets.at(index).size()*2;

        CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,1, 2,2,1,4,3,3>(
            new CalibrationFunctor(origin_target,targets[index],n_d,mode), ceres::TAKE_OWNERSHIP, num_residual 
        );
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(2.0), &curr_r,fs,cs,&skew, distorsion, Es.at(index).rot.data(),Es.at(index).trans.data() );

    }

    if(fix_radus){
        problem.SetParameterBlockConstant(&curr_r);
    }
    // else{
    //     problem.SetParameterLowerBound(&curr_r, 0, 0);
    // }


    // Run the solver!
    Solver::Options options;
    // printf("original iter : %d\n", options.max_num_iterations);
    options.max_num_iterations=200;

    // options.line_search_direction_type=ceres::LineSearchDirectionType::BFGS;
    options.function_tolerance=1.0e-8;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    // printf("final error: %f\n",summary.final_cost);
    // std::cout<<summary.FullReport()<<std::endl;
    if(curr_r<0) curr_r = -curr_r;
    Params results={fs[0],fs[1],cs[0],cs[1],skew,distorsion[0],distorsion[1],distorsion[2],distorsion[3]};
    results.radius = curr_r;

    normalize_Es();

    return results;
}

double Calibrator::cal_reprojection_error(std::vector<int> sample,Params params, int mode){
    double fx{params.fx}, fy{params.fy}, cx{params.cx}, cy{params.cy}, skew{params.skew};
    vector<double> ds = {1, params.d[0], params.d[1],params.d[2],params.d[3]};
    Eigen::Matrix3d E, E_inv, Qn;

    MomentsTracker* tracker = new MomentsTracker(n_d);

    double error_total=0;
    int n = sample.size();
    for(int i=0; i<n;i++){
        int index = sample[i];
        E= LieAlgebra::to_E(Es[index]);
        E_inv=  E.inverse();
        double t_error=0;
        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;
            Eigen::Matrix3d Wc;
            Wc <<   1.0, 0.0, -wx,
                    0.0, 1.0, -wy,
                    -wx, -wy, pow(wx,2)+pow(wy,2)-pow(params.radius,2);

            Point dp(0,0);
            if(mode ==0){
                Qn = E_inv.transpose()*Wc*E_inv;
                dp = tracker->ne2dp(Qn,ds);
            }
            else if(mode == 1){
                Qn = E_inv.transpose()*Wc*E_inv;
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

            double u_e = dp.x*fx+dp.y*skew+cx; 
            double v_e = dp.y*fy+cy;

            double u_o = targets[index][j].x;
            double v_o = targets[index][j].y;

            double tt_error= sqrt(pow(u_e-u_o,2)+pow(v_e-v_o,2));  
            t_error += tt_error;           
        }
        error_total += t_error/origin_target.size();
    }

    delete tracker;

    return error_total/n;
    
}
void Calibrator::normalize_Es(){
    for(int i=0; i<Es.size();i++){
        if(Es[i].trans(2)<0){
            Es[i].trans = -Es[i].trans;
            Eigen::Matrix3d R = LieAlgebra::to_SO3(Es[i].rot);
            R.col(0) = -R.col(0);
            R.col(1) = -R.col(1);
            Es[i].rot= LieAlgebra::to_so3(R);
        }
    }

}
void Calibrator::update_extrinsic(Params params){
    cv::Mat distCoeffs = cv::Mat::zeros(1,4, CV_64FC1);	
    cv::Mat intrinsic = cv::Mat::zeros(3,3, CV_64FC1);
    for(int i=0;i<4;i++){
        distCoeffs.at<double>(i)=params.d[i];
    }
    intrinsic.at<double>(0,0)=params.fx;
    intrinsic.at<double>(1,1)=params.fy;
    intrinsic.at<double>(0,2)=params.cx;
    intrinsic.at<double>(1,2)=params.cy;
    intrinsic.at<double>(0,1)=params.skew;
    intrinsic.at<double>(2,2)=1;

    for(int i=0;i<num_scene;i++){
        cv::Mat dpoints(targets[i].size(), 2, CV_64FC1, targets[i].data());
        cv::Mat udpoints(targets[i].size(), 2, CV_64FC1);

        cv::undistortPoints(dpoints,udpoints,intrinsic, distCoeffs);
        vector<Point> pts;
        for(int j=0;j<targets[i].size();j++){
            pts.push_back(Point(udpoints.at<double>(j,0),udpoints.at<double>(j,1)));
        }
        Eigen::Matrix3d E = get_H(pts).first;
        Es[i] = LieAlgebra::to_se3(E);
    }
    normalize_Es();
    
}

void Calibrator::get_extrinsic(string path){
    std::ofstream writeFile(path.data());
    for(int i=0; i<Es.size();i++){
        se3 E = Es[i];
        if(writeFile.is_open()){
            writeFile<<E.trans(0)<<"\t"<<E.trans(1)<<"\t"<<E.trans(2)<<"\t"<<E.rot(0)<<"\t"<<E.rot(1)<<"\t"<<E.rot(2)<<"\n";  
        }      
    }
    writeFile.close();
}
vector<se3> Calibrator::get_extrinsic(){
    return Es;
}

void Calibrator::printParams(Params p, bool full){
    std::cout<<"camera parameters:\n";
    
    if(full){
       printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n",p.fx,p.fy,p.cx,p.cy,p.skew,p.d[0],p.d[1],p.d[2],p.d[3]);
    }
    else{
        printf("%f\t%f\t%f\t%f\t%f\n",p.fx,p.fy,p.cx,p.cy,p.skew);
    }

}

Params Calibrator::calibrate(int mode, int width , int height){

    double alpha;
    std::cout.precision(12);
    Params initial_params{0.};

    printf("----------calibration mode: %d-------------\n", mode);
    init();
    printf("<before optimization>\n");
    printf("initial ");
    
    double fov= 90;
    bool result= cal_initial_params(&initial_params);
    if(initial_params.cx<0 || initial_params.cy <0){
        initial_params.fx = width/(2*tan(fov*M_PI/360));
        initial_params.fy = width/(2*tan(fov*M_PI/360));
        initial_params.skew= 0;
        initial_params.cx = width/2;
        initial_params.cy = height/2;
        initial_params.radius=original_r;
    }
    printParams(initial_params,true);

    set_inital_Es(initial_params);
    std::vector<int> sample = get_randomSample(num_scene,num_scene);

    printf("<start optimization>\n");
    Params final_params;
    double rperror;
    if(mode==0){
        initial_params= batch_optimize(sample, initial_params,true,2); // phase(fix parameter), mode(point, moment,...)
        printf(">coarse ");
        printParams(initial_params,true);
        // rperror = cal_reprojection_error(sample,initial_params,2);
        // printf("Estimated radius: %f, Reprojection error: %f\n",initial_params.radius,rperror);

        printf("It will take few minites.....\n");
        final_params= batch_optimize(sample, initial_params,false,mode);
        printf(">fine ");
        printParams(final_params,true);

        rperror = cal_reprojection_error(sample,final_params,mode);
        // double quality_logit = abs(1- final_params.radius/original_r);
        // double quality = 100*exp(-quality_logit);
        double quality = 100/(1+abs(log(final_params.radius/original_r)));
        printf("Calibration Quality: %f %%\n", quality);
        printf("Estimated radius: %f, Reprojection error: %f\n",final_params.radius,rperror);

        if(quality<80){
            cout<< "Warning: Calibration quality is so low!!"<<endl;
            cout<< "Check" <<endl;
            cout<< "\t1.Image qualites\t 2.Camera model"<<endl;
            cout<< "Start fixed-radius version..." <<endl;
            final_params= batch_optimize(sample, initial_params,true,mode);
            printf(">final ");
            printParams(final_params,true);
            rperror = cal_reprojection_error(sample,final_params,mode);
            printf("Fixed radius: %f, Reprojection error: %f\n",final_params.radius,rperror);
        }

    }
    else{
        final_params= batch_optimize(sample, initial_params,true,mode);
        printf(">fine ");
        printParams(final_params,true);
        rperror = cal_reprojection_error(sample,final_params,mode);
        printf("Reprojection error: %f\n",rperror);
    }
    return final_params;
}