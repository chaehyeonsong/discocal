#include "CCalibrator.h"

Calibrator::Calibrator(int n_x, int n_y,int n_d, double r,double distance, int max_scene, string results_path){
    this->n_x = n_x;
    this->n_y = n_y;
    this->n_d = n_d;
    this->max_scene = max_scene;
    this->num_scene = 0;
    this->thr_homography = 0;
    if(endsWith(results_path, "/")) this-> root_dir = results_path;
    else this-> root_dir= results_path+"/";


    this->original_r = r;
    this->distance = distance;

    // origin_conics.reserve(n_x*n_y);
    origin_target.reserve(n_x*n_y);
    targets.reserve(max_scene);
    ud_targets.reserve(max_scene);
    Hs.reserve(max_scene);
    Es.reserve(max_scene);
    images.reserve(max_scene);

    set_origin_target();
}
int Calibrator::get_num_scene(){
    return this->num_scene;
}

void Calibrator::init(){
    //Todo: for continous calibration
    // targets.clear();
    Es.clear();
    curr_r= original_r;
    // for(int i=0;i<num_scene;i++){
    //     rs[i]=original_r;
    // }
    // Hs.clear();
}

void Calibrator::set_origin_target(){

    for(int j=0;j<n_y;j++){
        for(int i=0;i<n_x;i++){
            double cx,cy;
            cx = distance*(i+1);
            cy = distance*(n_y-j);
            origin_target.push_back(Shape(cx,cy));
        }
    }

    ori_ms = get_mean_sigma(origin_target);
}

void Calibrator::inputImage(cv::Mat img){
    images.push_back(img);
}

void Calibrator::inputTarget(vector<Shape> target){
    // vector<Point> target2;
    // int n = target.size();
    // target2.reserve(n);
    // for(int i=0;i<n;i++){
    //     target2.push_back(Point(target[i].x,target[i].y));
    // }
    // targets.push_back(target2);
    // Hs.push_back(get_H(target2));

    targets.push_back(target);
    Hs.push_back(get_H(target));
    num_scene++;
    printf("%dth scene stored\n",num_scene);
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

Params Calibrator::batch_optimize(std::vector<int> sample, Params initial_params, int mode, bool fix_r, bool save_jacob, bool fix_intrinsic){

    double fs[]={initial_params.fx, initial_params.fy};
    double cs[]={initial_params.cx, initial_params.cy};
    double skew = initial_params.skew;
    double distorsion[4]={initial_params.d[0],initial_params.d[1],initial_params.d[2],initial_params.d[3]};



    // int n=sample.size();
    // vector<CostFunction*> cost_functions;
    // cost_functions.reserve(n);

    bool use_weight= false;
    Problem problem; 

    for(int i=0;i<sample.size();i++){
        int index =sample.at(i);
        int num_residual= targets.at(index).size()*2;

        CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,1, 2,2,1,4,3,3>(
            new CalibrationFunctor(origin_target,targets[index],n_d,mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual 
        );
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(2.0), &curr_r,fs,cs,&skew, distorsion, Es.at(index).rot.data(),Es.at(index).trans.data() );
        // problem.AddResidualBlock(cost_function, new ceres::TrivialLoss(), &curr_r,fs,cs,&skew, distorsion, Es.at(index).rot.data(),Es.at(index).trans.data() );

    }
    if(fix_r){
        // for(int i : sample){
        //     problem.SetParameterBlockConstant(&rs[i]);
        // }
        problem.SetParameterBlockConstant(&curr_r);
        // problem.SetParameterBlockConstant(&skew);
    }
    if(fix_intrinsic){
        problem.SetParameterBlockConstant(fs);
        problem.SetParameterBlockConstant(cs);
        problem.SetParameterBlockConstant(&skew);
        problem.SetParameterBlockConstant(distorsion);
    }


    // Run the solver!
    Solver::Options options;
    // printf("original iter : %d\n", options.max_num_iterations);
    // options.max_num_iterations=200;

    // options.line_search_direction_type=ceres::LineSearchDirectionType::BFGS;
    // options.function_tolerance=1.0e-8;
    options.minimizer_progress_to_stdout = false;
    if(save_jacob){
        options.minimizer_progress_to_stdout = true;
    }
    Solver::Summary summary;
    

    // summary.FullReport()
    // printf("final error: %f\n",summary.final_cost);
    // std::vector<double*> Problem::EvaluateOptions::parameter_blocks;
    Problem::EvaluateOptions eval_option;
    // eval_option.parameter_blocks={};

    Solve(options, &problem, &summary);

    if(save_jacob){
        double cost = 0.0;
        std::vector<double> residuals;
        std::vector<double> gradient;
        ceres::CRSMatrix jacobian;
        problem.Evaluate(eval_option, &cost, &residuals, &gradient, &jacobian);
        // cout<<cost<<endl;
        cout<<summary.FullReport()<<endl;
        std::vector<array<double,6>> extrinsic_gradient; 
        int n_row, n_col;
        string path;

        path = root_dir+ "gradient.txt";
        std::ofstream writeFile1(path.data());
        if(writeFile1.is_open()){
            for(int i =0;i <gradient.size();i++){
                    writeFile1<< gradient[i];
                    if(i == gradient.size()-1)writeFile1 << "\n";
                    else writeFile1 << "\t";
            }
        }
        writeFile1.close();


        n_row=jacobian.num_rows;
        n_col=jacobian.num_cols;
        Eigen::MatrixXd J(n_row, n_col);
        J.setZero();
        for(int i=0;i<n_row;i++){
            for(int curr_pos = jacobian.rows[i]; curr_pos <jacobian.rows[i+1]; curr_pos++){
                double value = jacobian.values[curr_pos];
                int col = jacobian.cols[curr_pos];
                J(i,col) = value;
            }
        }
        // cout<<n_row<<" "<<n_col<<endl;
        path = root_dir+"jacob.txt";
        std::ofstream writeFile2(path.data());
        if(writeFile2.is_open()){
            for(int i =0;i <n_row;i++){
                for(int j=0;j<n_col;j++){
                    writeFile2<< J(i,j);
                    if(j==n_col-1) writeFile2 << "\n";
                    else writeFile2 << "\t";
                }
            }
        }
        writeFile2.close();

        path = root_dir+"info.txt";
        std::ofstream writeFile3(path.data());
        if(writeFile3.is_open()){
            for(int i =0;i <targets.size();i++){
                vector<Shape> target_i = targets[i];
                for(int j=0; j<target_i.size();j++){
                     writeFile3 << target_i[j].Kxx << "\t" << target_i[j].Kxy << "\t" << target_i[j].Kyy <<"\t" <<to_string(int(target_i[j].area/1000)) << "\n";
                }
            }
        }
        writeFile3.close();
    }



    Params results(fs[0],fs[1],cs[0],cs[1],skew,distorsion[0],distorsion[1],distorsion[2],distorsion[3]);
    results.radius = curr_r;

    normalize_Es();

    return results;
}

void Calibrator::visualize_rep(string path,Params params, int mode){
    std::vector<int> sample;
    for(int i=0;i<num_scene;i++){
        sample.push_back(i);
    }
    // cv::Mat img = cv::Mat(930,1200,CV_8UC1,cv::Scalar(255));
    // int scale = 20;

    double fx{params.fx}, fy{params.fy}, cx{params.cx}, cy{params.cy}, skew{params.skew};
    vector<double> ds = {1, params.d[0], params.d[1],params.d[2],params.d[3]};
    Eigen::Matrix3d E, E_inv, Qn;

    MomentsTracker* tracker = new MomentsTracker(n_d);

    int n = sample.size();
    vector<array<double,4>> buffer;
    for(int i=0; i<n;i++){
        int index = sample[i];
        E= LieAlgebra::to_E(Es[index]);

        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;
            Point p_i = tracker->project(wx,wy,curr_r,params,E,mode);
            double u_o = targets[index][j].x;
            double v_o = targets[index][j].y;
            buffer.push_back(array<double,4>{u_o,v_o,p_i.x,p_i.y});
            // double u_diff= (u_e-u_o);
            // double v_diff = (v_e-v_o);
            // cv::arrowedLine(img,cv::Point2d(u_o, v_o),cv::Point2d(u_o+u_diff,v_o+v_diff),cv::Scalar(0),1,8,0,0.2);
        }
    }
    delete tracker;

    // cv::imwrite(path,img);
    std::ofstream writeFile(path.data());
    for(int i=0; i<buffer.size();i++){
        if(writeFile.is_open()){
            writeFile<<buffer[i][0]<<"\t"<<buffer[i][1]<<"\t"<<buffer[i][2]<<"\t"<<buffer[i][3]<<"\n";  
        }      
    }
    writeFile.close();
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
        double t_error=0;
        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;
            Point p_i = tracker->project(wx,wy,curr_r,params,E,mode);

            double u_e = p_i.x; 
            double v_e = p_i.y;

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

void Calibrator::save_extrinsic(){
    save_extrinsic(root_dir+"est_pose.txt");
}
void Calibrator::save_extrinsic(string path){
    std::ofstream writeFile(path.data());
    for(int i=0; i<Es.size();i++){
        se3 E = Es[i];
        if(writeFile.is_open()){
            writeFile<<E.rot(0)<<"\t"<<E.rot(1)<<"\t"<<E.rot(2)<<"\t"<<E.trans(0)<<"\t"<<E.trans(1)<<"\t"<<E.trans(2)<<"\n";  
        }      
    }
    writeFile.close();
}
vector<se3> Calibrator::get_extrinsic(){
    return Es;
}

void Calibrator::update_Es(Params intrinsic, int mode, bool fix_radius,bool save_jacob){
    init();
    set_inital_Es(intrinsic);
    std::vector<int> sample = random_sampling(num_scene,num_scene);
    curr_r = original_r;
    batch_optimize(sample, intrinsic,mode, fix_radius,save_jacob,true);
}


Params Calibrator::calibrate(int mode, bool fix_radius, bool save_jacob){
    printf("----------calibration mode: %d-------------\n", mode);
    init();
    bool debug = true;
    double alpha;
    std::cout.precision(12);
    Params initial_params;
    Params final_params;
    
   
    bool result= cal_initial_params(&initial_params);
     // if(debug) std::cout<<"<zhang's method>\n";
    if(debug) printParams(initial_params,true);
    
    printf("optimization mode: %d\n", mode);


    // 0: all, 2: w/o r
    struct timeval  tv;
	double begin, end;
    gettimeofday(&tv, NULL);
	begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;

    set_inital_Es(initial_params);
    std::vector<int> sample = random_sampling(num_scene,num_scene);
    // for(int i=0; i<sample.size();i++){
    //     rs.push_back(original_r);
    // }
    if(mode==0){
        // initial_params.skew=0;
        final_params= batch_optimize(sample, initial_params,2);
        printParams(final_params,true);
        double rperror = cal_reprojection_error(sample,final_params,2);
        printf("Estimated radius: %f, Reprojection error: %f\n",final_params.radius,rperror);

        printf(">>>fine\n");
        final_params= batch_optimize(sample, final_params,mode, fix_radius,save_jacob);
        printParams(final_params,true);
        double quality = abs(1- final_params.radius/original_r);
        rperror = cal_reprojection_error(sample,final_params,mode);
        printf("Calibration Quality: %f %%\n", 100*exp(-quality));
        printf("Estimated radius: %f, Reprojection error: %f\n",final_params.radius,rperror);
    }
    else{
        final_params= batch_optimize(sample, initial_params,mode,fix_radius,save_jacob);
        
    }


    // visualize_rep("../results/rep"+to_string(mode)+".png",final_params,mode);
    
    // undistort_total_images(final_params);
    gettimeofday(&tv, NULL);
    end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    double duration =(end - begin) / 1000;
    printf("finishied. Runtime: %.2f\n", duration);

    return final_params;
}


void Calibrator::batch_calibrate(int mode){
    std::cout.precision(12);
    Params initial_params;
    Params final_params;
    
    int phase = 2;
    // if(mode !=0) phase = 2;

    int total_iter = 30;
    int batch_scene=30;

    array<vector<double>,6> buffer;
    std::vector<int> real_sample;
    int t_fail;
    double total_fail{0};

    bool result= cal_initial_params(&initial_params);

    // clock_t start_t, finish_t;
    struct timeval  tv;
	double begin, end;
    vector<double> durations;

    for(int i=0; i<total_iter;i++){
        // start_t = clock();
        gettimeofday(&tv, NULL);
	    begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;

        init();
        set_inital_Es(initial_params);
        std::vector<int> sample = random_sampling(max_scene,batch_scene);
        real_sample.clear();
        t_fail=0;
        for(int j=0; j<batch_scene;j++){
            if(sample[j]<num_scene) real_sample.push_back(sample[j]);
            else {
                t_fail++;
            }
        }

        // for(int k=0; k<batch_scene;k++) printf("%d\t",sample[k]);
        // printf("\n");

        curr_r = original_r;
        // for(int i=0; i<real_sample.size();i++){
        //     rs.push_back(original_r);
        // }
        if(mode==0){
            final_params= batch_optimize(real_sample, initial_params,2,2);
            final_params= batch_optimize(real_sample, final_params,2,mode);
        }
        else if (mode ==5){
            double error = 10;
            int iter=0;
            for(int iter =0;iter<10;iter++){
                Params prev_params = final_params;
                final_params= batch_optimize(real_sample, final_params,2,2);
                error = sqrt(pow(final_params.fx-prev_params.fx,2)+pow(final_params.fy-prev_params.fy,2)+pow(final_params.cx-prev_params.cx,2)+pow(final_params.cy-prev_params.cy,2));
                printf(">> %d semi-iter--> error: %f\n",iter, error);
                if(error<0.01) break;
                update_control_point(real_sample, final_params);
            }
        }
        else{
            final_params= batch_optimize(real_sample, initial_params,2,mode);
        }
        buffer[0].push_back(final_params.fx);
        buffer[1].push_back(final_params.fy);
        buffer[2].push_back(final_params.cx);
        buffer[3].push_back(final_params.cy);
        buffer[4].push_back(final_params.d[0]);
        buffer[5].push_back(final_params.d[1]);
        total_fail+= t_fail;
        
        // finish_t = clock();
        // double duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
        gettimeofday(&tv, NULL);
	    end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
        double duration =(end - begin) / 1000;
        printf("iter %d finishied. Runtime: %.2f\n", i+1,duration);
        durations.push_back(duration);
    }

    

    array<double, 6> means, stds;
    double time_m, time_s;
    for(int i=0;i<total_iter;i++){
        time_m += durations[i];
        time_s += pow(durations[i],2);
    }
    time_m /= total_iter;
    time_s = sqrt(time_s/total_iter - pow(time_m,2));

    for(int i=0;i<6;i++){
        means[i]=0;
        stds[i]=0;
        for(int j=0;j<total_iter;j++){
            means[i]+= buffer[i][j];
            stds[i] += pow( buffer[i][j],2);
        }
        means[i] /= total_iter;
        stds[i] = sqrt(stds[i]/total_iter - pow(means[i],2));
        // printf("mean: %f, std: %f\n",means[i],stds[i]);
    }
    printf("----------optimization mode: %d----------\n", mode);
    printf("mean: %f, %f, %f, %f, %f, %f\n",means[0],means[1],means[2],means[3],means[4],means[5]);
    printf("std: %f, %f, %f, %f, %f, %f\n",stds[0],stds[1],stds[2],stds[3],stds[4],stds[5]);

    printf("latex: %.1f$\\pm$%.2f &%.1f$\\pm$%.2f &%.1f$\\pm$%.2f &%.1f$\\pm$%.2f &%.1f$\\pm$%.2f &%.1f$\\pm$%.2f \n",means[0],stds[0],means[1],stds[1],means[2],stds[2],means[3],stds[3],means[4],stds[4],means[5],stds[5]);
    printf("mean_fail: %f\n", total_fail/total_iter);

    printf("duration mean & std: %.2f , %.4f\n",time_m, time_s);



}

//---------------------------------------------------------------------------------------------------------------------------------//

void Calibrator::update_control_point(std::vector<int> real_sample,Params results){
    vector<double> distortion{1, results.d[0],results.d[1],results.d[2],results.d[3] };
    cv::Mat distCoeffs = cv::Mat(1,4, CV_64FC1, results.d);	
    cv::Mat intrinsic = cv::Mat::zeros(3,3, CV_64FC1);
    cv::Mat K = cv::Mat::zeros(3,3, CV_64FC1);

    Eigen::Matrix3d K1;
    K1<< results.fx, results.skew, results.cx,
            0, results.fy, results.cy,
            0, 0, 1;

    intrinsic.at<double>(0,0)=results.fx;
    intrinsic.at<double>(0,1)=results.skew;
    intrinsic.at<double>(0,2)=results.cx;
    intrinsic.at<double>(1,1)=results.fy;
    intrinsic.at<double>(1,2)=results.cy;
    intrinsic.at<double>(2,2)=1;

    double W= 1200;
    double H = 900;
    Eigen::Matrix3d H_final, K2, H_backward;
    K2 << 200/distance, 0, 100,
        0, -200/distance, H-50,
        0, 0, 1;
    

    TargetDetector detector(n_x, n_y,false);
    MomentsTracker tracker(n_d);
    for(int iter=0;iter<real_sample.size();iter++){
        int i = real_sample[iter];
        cv::Mat img = images[i];
        cv::Mat canonical_img;
        cv::Mat undist_img;
        int w= img.cols;
        int h = img.rows;

        H_final = K2*(LieAlgebra::to_E(Es[i]).inverse())*K1.inverse();
        H_backward = LieAlgebra::to_E(Es[i])*K2.inverse();
        
        K.at<double>(0,0)=H_final(0,0);
        K.at<double>(0,1)=H_final(0,1);
        K.at<double>(0,2)=H_final(0,2);
        K.at<double>(1,0)=H_final(1,0);
        K.at<double>(1,1)=H_final(1,1);
        K.at<double>(1,2)=H_final(1,2);
        K.at<double>(2,0)=H_final(2,0);
        K.at<double>(2,1)=H_final(2,1);
        K.at<double>(2,2)=H_final(2,2);

        cv::undistort(img, undist_img,intrinsic,distCoeffs,intrinsic);
        cv::warpPerspective(undist_img, canonical_img, K, cv::Size(w, h));
        // cv::imwrite("../temp.png",canonical_img);
        images[i] = canonical_img;
        
        auto d_results = detector.detect(canonical_img, "circle");
        if(d_results.first){
            vector<Shape> target = d_results.second;
            vector<Shape> target2;
            int n = target.size();
            target2.reserve(n);
            for(int j=0;j<n;j++){
                // printf("x: %f, y: %f\n",targets[i][j].x,targets[i][j].y);
                Eigen::Vector3d pn{target[j].x, target[j].y, 1};
                pn = H_backward*pn;
                Point p_n = Point(pn(0)/pn(2), pn(1)/pn(2));
                Point dp = tracker.distort_Point(p_n,distortion);
                double u_e = dp.x*results.fx+dp.y*results.skew+results.cx; 
                double v_e = dp.y*results.fy+results.cy;
                target2.push_back(Shape(u_e,v_e));
                // printf("x: %f, y: %f\n",u_e,v_e);
            }
            targets[i]=target2;
            // Hs[i]=get_H(target2);
        }

    }
}




//---------------------Zhang's method-----------------------------//

array<double, 4> Calibrator::get_mean_sigma(const vector<Shape> &target){
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

pair<Eigen::Matrix3d,double> Calibrator::get_H(const vector<Shape> &target){
    
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
    // std::cout<<"num_scene: "<<this->num_scene<<std::endl;

    double h_mean=0;
    for(int i=0;i<num_scene;i++){
        h_mean+= Hs.at(i).second;
    }
    h_mean = h_mean/num_scene;
    // printf("h_sigma: %f\n",h_sigma);
    
    for(int i=0;i<num_scene;i++){
        pair<Eigen::Matrix3d,double> result  = Hs.at(i);
        Eigen::Matrix3d H = result.first;
        // cout<<H<<endl;
        double h_ratio =result.second/h_mean;
        // std::cout<<"singular value ratio: "<<h_ratio<<std::endl;
        H*= h_ratio;
        if (h_ratio>this->thr_homography){
            Eigen::VectorXd v_01 = get_Vij(H,0,1);
            Eigen::VectorXd v_00 = get_Vij(H,0,0);
            Eigen::VectorXd v_11 = get_Vij(H,1,1);
            X.row(2*i)=v_01;
            X.row(2*i+1) = v_00-v_11;
        }
    }
    // cout<<X<<endl;
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

//---------------------Zhang's method-----------------------------//