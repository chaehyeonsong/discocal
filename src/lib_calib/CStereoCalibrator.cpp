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


void StereoCalibrator::calibrate(
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

    for (int camera_index = 0; camera_index < all_targets.size(); camera_index++) {
        // int camera_index=0;
        // Params param = all_params[camera_index];
        // double fcs[5]={param.fx, param.fy, param.cx, param.cy, param.skew};
        // double distorsion[4]={param.d[0],param.d[1],param.d[2],param.d[3]};
        vector<Shape> original_targets = cal_origin_target(all_radius[camera_index]);
        
        for(int scene_index=0;scene_index<all_targets[camera_index].size();scene_index++){
            if (!all_targets[0].at(scene_index).first) continue;
            if(camera_index==0){
                CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,5,4,3,3>(
                    new CalibrationFunctor(original_targets,all_targets[camera_index][scene_index].second,all_radius[camera_index],all_n_d[camera_index],mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual
                );
                problem.AddResidualBlock(cost_function, loss_function, all_params[camera_index].K,  all_params[camera_index].d, Es.at(scene_index).rot.data(),Es.at(scene_index).trans.data() );
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
    // cout << summary.FullReport() << endl;

    normalize_Es(Es);    
    normalize_Es(all_ext);
}

