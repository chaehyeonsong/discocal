#include "CStereoCalibrator.h"


StereoCalibrator::StereoCalibrator(int n_x, int n_y, bool asymmetric, double distance){
    this->n_x = n_x;
    this->n_y = n_y;
    this->is_asymmetric = asymmetric;
    // if(endsWith(results_path, "/")) this-> root_dir = results_path;
    // else this-> root_dir= results_path+"/";

    this->distance = distance;
}


vector<Shape> StereoCalibrator::cal_origin_target(double radius){
    vector<Shape> origin_target;
    origin_target.reserve(n_x*n_y);
    if(is_asymmetric){
        for(int i=0;i<n_x;i++){
            double offset =0;
            if(i%2==0) offset= distance/2.0;
            for(int j=0;j<n_y;j++){
                double x,y;
                x = distance*i/2.0;
                y = distance*j+offset;
                Shape target(x,y, M_PI*radius*radius);
                // target.radius = original_r;
                origin_target.push_back(target);
            }
        }
    }
    else{
        for(int j=0;j<n_y;j++){
            for(int i=0;i<n_x;i++){
                double x,y;
                x = distance*(i+1);
                y = distance*(n_y-j);
                Shape target(x,y, M_PI*radius*radius);
                // target.radius = original_r;
                origin_target.push_back(target);
            }
        }
    }
    return origin_target;
}

void StereoCalibrator::normalize_Es(vector<se3>& Es){
    for(int i=0; i<Es.size();i++){
        Es[i].rot = LieAlgebra::normalize_so3(Es[i].rot);
        if(Es[i].trans(2)<0){
            Es[i].trans = -Es[i].trans;
            Eigen::Matrix3d R = LieAlgebra::to_SO3(Es[i].rot);
            R.col(0) = -R.col(0);
            R.col(1) = -R.col(1);
            Es[i].rot= LieAlgebra::to_so3(R);
        }
    }

}


vector<array<double, 6>> StereoCalibrator::calibrate(
    const vector<vector<Target>> & all_targets,
    vector<se3>& Es,
    vector<se3>& all_ext,
    vector<Params>& all_params,
    const vector<double>& all_radius,
    const vector<double>& all_n_d,
    bool full_optimize
){
    int mode = 0; //0: moments, 2: point
    Problem problem; 
    ceres::LossFunction* loss_function; 
    bool use_weight= true;
    loss_function = new ceres::TrivialLoss();
    int num_residual= n_x * n_y *2;
    int n_col_real = 0;
    int ext_start_index = 0;
    for (int camera_index = 0; camera_index < all_targets.size(); camera_index++) {
        // int camera_index=0;
        // Params param = all_params[camera_index];
        // double fcs[5]={param.fx, param.fy, param.cx, param.cy, param.skew};
        // double distorsion[4]={param.d[0],param.d[1],param.d[2],param.d[3]};
        vector<Shape> original_targets = cal_origin_target(all_radius[camera_index]);

        if(camera_index==0){
            n_col_real += full_optimize?  5 + all_n_d[camera_index] : 0;
        }
        else{
            if(camera_index==1) ext_start_index = n_col_real;
            n_col_real += full_optimize?  5 + all_n_d[camera_index] + 6 : 6;
        }
        
        for(int scene_index=0;scene_index<all_targets[camera_index].size();scene_index++){
            if (!all_targets[0].at(scene_index).first) continue;
            if(camera_index==0){
                CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,5,4,3,3>(
                    new CalibrationFunctor(original_targets,all_targets[camera_index][scene_index].second,all_radius[camera_index],all_n_d[camera_index],mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual
                );
                problem.AddResidualBlock(cost_function, loss_function, all_params[camera_index].K,  all_params[camera_index].d, Es.at(scene_index).rot.data(),Es.at(scene_index).trans.data() );
                n_col_real += 6;
            }
            else{
                if (!all_targets.at(camera_index).at(scene_index).first) continue;
                CostFunction* cost_function = new NumericDiffCostFunction<SubCalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,5,4,3,3,3,3>(
                    new SubCalibrationFunctor(original_targets,all_targets[camera_index][scene_index].second,all_radius[camera_index],all_n_d[camera_index],mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual
                );
                problem.AddResidualBlock(cost_function, loss_function,  all_params[camera_index].K,  all_params[camera_index].d, 
                    Es.at(scene_index).rot.data(),Es.at(scene_index).trans.data() , all_ext.at(camera_index - 1).rot.data(), all_ext.at(camera_index - 1).trans.data());
            }
        }



        if (!full_optimize){
            problem.SetParameterBlockConstant(all_params[camera_index].K);
            problem.SetParameterBlockConstant(all_params[camera_index].d);
        }
    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    normalize_Es(Es);    
    normalize_Es(all_ext);

    double cost = 0.0;
    Problem::EvaluateOptions eval_option;
    std::vector<double> residuals;
    std::vector<double> gradient;
    ceres::CRSMatrix jacobian;
    problem.Evaluate(eval_option, &cost, &residuals, &gradient, &jacobian);

    int n_row, n_col;
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

    string path;

    // erase constant parameter columns
    Eigen::MatrixXd J_real(n_row, n_col_real);
    J_real.setZero();
    int col_pts= 0;
    for(int j=0;j<n_col;j++){
        if(col_pts >= n_col_real){
            std::cerr<<"Error in Jacobian extraction: expected cols "<< n_col_real << " but got "<< col_pts <<std::endl;
        }
        if(J.col(j).isZero()) continue;
        else{
            J_real.col(col_pts)= J.col(j);
            col_pts++;
        }
    }

    // string temp_path = "jacob.txt";
    // std::ofstream writeFile2(temp_path.data());
    // if(writeFile2.is_open()){
    //     for(int i =0;i <n_row;i++){
    //         for(int j=0;j<n_col_real;j++){
    //             writeFile2<< J_real(i,j);
    //             if(j==n_col_real-1) writeFile2 << "\n";
    //             else writeFile2 << "\t";
    //         }
    //     }
    // }
    // writeFile2.close();

   
    // build measurement covariance matrix
    // Eigen::MatrixXd M_cov(n_row, n_row);
    // M_cov.setZero();
    // int scene_count =0;
    // for(int camera_index = 0; camera_index < all_targets.size(); camera_index++){
    //     for(int scene_index=0; scene_index<all_targets[camera_index].size();scene_index++){
    //         Target target_i = all_targets[camera_index][scene_index];
    //         if (!target_i.first) continue;
    //         for(int j=0; j<n_x*n_y;j++){
    //             int index = (scene_count*n_x*n_y+j)*2;
    //             Shape blob_ij = target_i.second[j];
    //             M_cov(index,index) = blob_ij.Kxx;
    //             M_cov(index+1,index) = blob_ij.Kxy;
    //             M_cov(index,index+1) = blob_ij.Kxy;
    //             M_cov(index+1,index+1) = blob_ij.Kyy;
    //         }
    //         scene_count++;
    //     }
    // }

    Eigen::MatrixXd info_matrix(n_col_real, n_col_real);
    info_matrix = J_real.transpose() * J_real;
    Eigen::MatrixXd  cov_matrix = info_matrix.inverse();
    vector<array<double, 6>>  ext_std;
    int start_index = ext_start_index;
    for(int camera_index = 1; camera_index < all_targets.size(); camera_index++){
        array<double, 6> ext_std_i{0,};
        if(full_optimize) start_index += 5 + all_n_d[camera_index];
        for(int i=0; i<6;i++){
            ext_std_i[i] = sqrt(cov_matrix(start_index+i , start_index+i));
        }
        start_index+=6;
        ext_std.push_back(ext_std_i);
    }

    // for(int i=0;i<ext_std.size();i++){
    //     cout<<"Extrinsic std dev for camera0to"<< i+1 <<": ";
    //     cout<< "["<< ext_std[i][0]<<", "<< ext_std[i][1]<<", "<< ext_std[i][2]<<", "<< ext_std[i][3]<<", "<< ext_std[i][4]<<", "<< ext_std[i][5]<<"]"<<endl;
    // }
    // cout << summary.FullReport() << endl;

    return ext_std;
}

