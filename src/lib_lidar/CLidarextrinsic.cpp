#include "CLidarextrinsic.h"

std::atomic<bool> continue_flag(false);
//main function -> estimate_lidar_extrinsic() : all other functions are called inside this function
bool estimate_lidar_extrinsic(const std::string& pcd_path,
                               const LidarCalibrationParams& params,
                               se3& out_pose, int cnt) {
    std::cout << "[INFO] Loading point cloud from: " << pcd_path << std::endl;

    // 1. ????? ????? ?��?
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcd_path.ends_with(".pcd")) {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
            std::cerr << "[ERROR] Couldn't read PCD file: " << pcd_path << std::endl;
            return false;
        }
    } else {
        std::cerr << "[ERROR] Unsupported file format: " << pcd_path << std::endl;
        return false;
    }

    std::cout << "[INFO] Loaded point cloud with " << cloud->size() << " points." << std::endl;

    applyBoundaryFilter(cloud, params, cloud);

    if (params.visualize) {
        std::string save_dir = "../sample_data/viz_check/cloud_" + std::to_string(cnt) + "/";
        std::filesystem::create_directories(save_dir);  

        std::string ply_path = save_dir + "filtered_cloud.ply";
        pcl::io::savePLYFileBinary(ply_path, *cloud);
        std::cout << "[INFO] Point cloud saved to: " << ply_path << std::endl;

        if (fork() == 0){
            if (!visualizeCloudInteractive(cloud, "Filtered Cloud")) std::exit(1);
            std::exit(0);
        } else{
            int status;
            wait(&status);
        }
    }
    auto cluster_indices = clusterPoints(cloud, params.eps, 10);
    if (cluster_indices.empty()) {
        std::cerr << "[ERROR] No clusters found in the point cloud." << std::endl;
        return false;
    }
    auto most_planar_cluster = findMostPlanarCluster(cloud, cluster_indices);
    if (params.visualize) {
        std::string save_dir = "../sample_data/viz_check/cloud_" + std::to_string(cnt) + "/";     
        std::filesystem::create_directories(save_dir);  

        std::string ply_path = save_dir + "most_planar_cluster.ply";
        pcl::io::savePLYFileBinary(ply_path, *most_planar_cluster);
        std::cout << "[INFO] Most planar cluster saved to: " << ply_path << std::endl;
        if (fork() == 0){
            if (!visualizeCloudInteractive(most_planar_cluster, "Most planar cluster")) std::exit(1);
            std::exit(0);
        } else{
            int status;
            wait(&status);
        }
    }
    //projection
    auto [ransac_plane_model, inliers] = detectPlanes(most_planar_cluster, params.distance_threshold, 3, params.max_iterations);
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(most_planar_cluster);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlier_cloud);
    Eigen::Vector4f plane_model = pcaPlaneFitting(inlier_cloud); 
    auto projected_cloud = projectPointsToPlane(inlier_cloud, plane_model);
    if(params.visualize) {
        std::string save_dir = "../sample_data/viz_check/cloud_" + std::to_string(cnt) + "/";  
        std::filesystem::create_directories(save_dir);  

        std::string ply_path = save_dir + "projected_cloud.ply";
        pcl::io::savePLYFileBinary(ply_path, *projected_cloud);
        std::cout << "[INFO] Projected cloud saved to: " << ply_path << std::endl;
        if (fork() == 0){
            if (!visualizeCloudInteractive(projected_cloud, "projected cloud"))  std::exit(1);
            std::exit(0);
        } else{
            int status;
            wait(&status);
        } 
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_pcd = findBoundaryPointsImproved(projected_cloud, 0.3*params.radius, 0.0, params.cdr, params.direction_var);
    if(params.visualize){
        std::string save_dir = "../sample_data/viz_check/cloud_" + std::to_string(cnt) + "/";  
        std::filesystem::create_directories(save_dir);  

        std::string ply_path = save_dir + "boundary_pcd.ply";
        pcl::io::savePLYFileBinary(ply_path, *boundary_pcd);
        std::cout << "[INFO] Boundary point cloud saved to: " << ply_path << std::endl;
        if (fork() == 0){
            if (!visualizeCloudInteractive(boundary_pcd, "boundary point clouds"))  std::exit(1);
            std::exit(0);
        } else{
            int status;
            wait(&status);
        } 
    }
    auto best_clusters = selectNCircleClusters(boundary_pcd, params.distance, params.radius, params.n_x * params.n_y);
    if (best_clusters.empty()) {
        std::cerr << "[ERROR] No suitable clusters found." << std::endl;
        return false;
    }
    if(params.visualize) {
        std::string save_dir = "../sample_data/viz_check/cloud_" + std::to_string(cnt) + "/";  
        std::filesystem::create_directories(save_dir);  

        for (size_t i = 0; i < best_clusters.size(); ++i) {
            std::string ply_path = save_dir + "best_cluster_" + std::to_string(i) + ".ply";
            pcl::io::savePLYFileBinary(ply_path, *best_clusters[i]);
            std::cout << "[INFO] Best cluster " << i << " saved to: " << ply_path << std::endl;
            
        }
        if (fork() == 0){
            if (!visualizeClustersInteractive(best_clusters, "best_clusters"))  std::exit(1);
            std::exit(0);
        } else{
            int status;
            wait(&status);
        } 
        
    }
    std::vector<Eigen::Vector3f> circle_centers;
    for (const auto& cluster : best_clusters) {
        auto [best_circle, best_radius, inliers] = ransacCircleFit(cluster, plane_model, 1000, 0.2* params.radius);
        auto [final_circle_center, final_radius] = circleFitConstrained2D(inliers, plane_model);
        circle_centers.push_back(final_circle_center);
    }
    cout<< "[INFO] Circle centers found: " << circle_centers.size() << endl;
    auto sorted_centers = sortCentersByRotatedLidarFrame(circle_centers, params.n_x, params.n_y);
    Eigen::Matrix4f T_board = optimize_rotation_transform_general(
        sorted_centers, plane_model, params.n_x, params.n_y, params.distance);
    if(params.visualize){
        
        for (const auto& center : circle_centers) {
            std::cout << "Circle center: " << center.transpose() << std::endl;
        }
        
        std::cout << "Sorted Centers:" << std::endl;
        for (const auto& center : sorted_centers) {
            std::cout << "x: " << center.x() << ", y: " << center.y() << ", z: " << center.z() << std::endl;
        }
        
        std::cout << "[INFO] Optimized transformation matrix:\n" << T_board << std::endl;
    }
    out_pose = LieAlgebra::to_se3(T_board.cast<double>().eval());
    return true;  
}


// Keyboard callback function
void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
    if (event.keyDown()) {
        std::string key = event.getKeySym();
        if (key == "c" || key == "C" ) {
            continue_flag = true;
        }
    }
}
//functions to visualize pcd
bool visualizeCloudInteractive(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &title) {
    continue_flag = false;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));

    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 255);
    viewer->addPointCloud(cloud, color_handler, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    viewer->resetCamera();

    viewer->registerKeyboardCallback(keyboardCallback);

    std::cout << "[INFO] Press 'c' to continue" << std::endl;

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (continue_flag ) {
            viewer->close();  // Close the window programmatically
            
            break;
        }
    }
    viewer.reset();  // Cleanup
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Optional small delay
    return true;
}


bool visualizeClustersInteractive(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, const std::string& title) {
    continue_flag = false;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));

    viewer->setBackgroundColor(0, 0, 0);
    for (size_t i = 0; i < clusters.size(); ++i) {
        std::string cloud_name = "cluster_" + std::to_string(i);

        // Generate unique color per cluster
        int r = (37 * (i+1)) % 255;
        int g = (67 * (i+1)) % 255;
        int b = (97 * (i+1)) % 255;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(clusters[i], r, g, b);
        viewer->addPointCloud<pcl::PointXYZ>(clusters[i], color_handler, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    }
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    viewer->resetCamera();

    viewer->registerKeyboardCallback(keyboardCallback);

    std::cout << "[INFO] Press 'c' to continue" << std::endl;

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (continue_flag ) {
            viewer->close();  // Close the window programmatically
            break;
        }
    }
    viewer.reset();  // Cleanup
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Optional small delay
    return true;
    
    
}

//coarse boundary filter func
void applyBoundaryFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const LidarCalibrationParams& params,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    
    pass.setFilterFieldName("x");
    pass.setFilterLimits(params.coarse_bd_x.first, params.coarse_bd_x.second);
    pass.filter(*filtered_cloud);

    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(params.coarse_bd_y.first, params.coarse_bd_y.second);
    pass.filter(*filtered_cloud);

    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(params.coarse_bd_z.first, params.coarse_bd_z.second);
    pass.filter(*filtered_cloud);
}
//first cluster the points and..
std::vector<pcl::PointIndices> clusterPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             float eps , int min_samples) {
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(eps);
    ec.setMinClusterSize(min_samples);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    return cluster_indices;
}
//find the most planar cluster -> board detect
pcl::PointCloud<pcl::PointXYZ>::Ptr findMostPlanarCluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices) {

    float min_variance_ratio = std::numeric_limits<float>::max();
    pcl::PointCloud<pcl::PointXYZ>::Ptr most_planar_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& indices : cluster_indices) {
        if (indices.indices.size() < 50) continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }

        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cluster);
        Eigen::Vector3f eigen_vals = pca.getEigenValues();

        float planar_ratio = (eigen_vals[2] / eigen_vals[0]) / eigen_vals[1];

        float min_z = cluster->points.front().z;
        float max_z = min_z;
        for (const auto& pt : cluster->points) {
            min_z = std::min(min_z, pt.z);
            max_z = std::max(max_z, pt.z);
        }

        if (planar_ratio < min_variance_ratio) {
            min_variance_ratio = planar_ratio;
            most_planar_cluster = cluster;
        }
    }

    return most_planar_cluster;
}
//detect most reliable plane inside the board cluster 
std::pair<Eigen::Vector4f, pcl::PointIndices::Ptr> detectPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                                double distance_threshold ,
                                                                int ransac_n , int num_iterations) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(num_iterations);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    Eigen::Vector4f plane_model(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    return std::make_pair(plane_model, inliers);
}

Eigen::Vector4f pcaPlaneFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance_matrix);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);

    float d = -centroid.head<3>().dot(normal);

    return Eigen::Vector4f(normal[0], normal[1], normal[2], d);
}

// Function to project points to a plane using the plane's normal and point on the plane
pcl::PointCloud<pcl::PointXYZ>::Ptr projectPointsToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                                                         const Eigen::Vector4f &plane_model) {
    Eigen::Vector3f normal = plane_model.head<3>();
    float d = plane_model[3];
    normal.normalize();

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &point : cloud->points) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        float t = -d / normal.dot(p);
        Eigen::Vector3f projected_point = t * p;
        projected_cloud->points.emplace_back(projected_point[0], projected_point[1], projected_point[2]);
    }
    return projected_cloud;
}

//found boundary points inside the projected pointcloud 
pcl::PointCloud<pcl::PointXYZ>::Ptr findBoundaryPointsImproved(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                               double radius, double min_distance, double cdr_threshold, double direction_var_threshold) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<bool> boundary_mask(cloud->points.size(), false);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_distance;
        if (kdtree.radiusSearch(cloud->points[i], radius, point_idx_radius_search, point_radius_squared_distance) > 0) {
            if (point_idx_radius_search.size() <= 1)
                continue;

            const auto &point = cloud->points[i];

            std::vector<Eigen::Vector3f> valid_neighbors;
            for (const auto &neighbor_idx : point_idx_radius_search) {
                const auto &neighbor = cloud->points[neighbor_idx];
                Eigen::Vector3f neighbor_vec(neighbor.x - point.x, neighbor.y - point.y, neighbor.z - point.z);
                if (neighbor_vec.norm() > min_distance) {
                    valid_neighbors.push_back(neighbor_vec);
                }
            }

            if (valid_neighbors.size() <= 1)
                continue;

            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (const auto &vec : valid_neighbors) {
                centroid += vec;
            }
            centroid /= valid_neighbors.size();

            double mean_distance = 0.0;
            for (const auto &vec : valid_neighbors) {
                mean_distance += vec.norm();
            }
            mean_distance /= valid_neighbors.size();

            double centroid_distance = centroid.norm();
            double cdr = centroid_distance / mean_distance;

            if (cdr <= cdr_threshold) {
                continue;
            }
            // Calculate variance manually
            Eigen::Vector3f mean_direction = Eigen::Vector3f::Zero();
            for (const auto &vec : valid_neighbors) {
                mean_direction += vec.normalized();
            }
            mean_direction /= valid_neighbors.size();

            Eigen::Vector3f direction_variance = Eigen::Vector3f::Zero();
            for (const auto &vec : valid_neighbors) {
                direction_variance += (vec.normalized() - mean_direction).cwiseAbs2();
            }
            direction_variance /= valid_neighbors.size();

            if (direction_variance.sum() < direction_var_threshold) {
                boundary_mask[i] = true;
            }
        }
    }

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (boundary_mask[i]) {
            boundary_cloud->points.push_back(cloud->points[i]);
        }
    }

    return boundary_cloud;
}
//select N circle clusters inside the boundarypoints
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
selectNCircleClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      double distance, double radius,
                      int n) 
    {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(distance-radius*2-0.025); 
    ec.setMinClusterSize(3); 
    ec.setMaxClusterSize(100);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    using ScoredCluster = std::pair<float, pcl::PointCloud<pcl::PointXYZ>::Ptr>;
    std::vector<ScoredCluster> scored_clusters;

    for (const auto& indices : cluster_indices) {
        auto cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        Eigen::Vector3f center = Eigen::Vector3f::Zero();

        for (int idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
            center += cloud->points[idx].getVector3fMap();
        }
        center /= cluster->points.size();

        float score = 0.0f;
        for (const auto& pt : cluster->points) {
            float d = (pt.getVector3fMap() - center).norm();
            score += std::pow(d - (radius), 2);
        }
        score /= cluster->points.size();

        scored_clusters.emplace_back(score, cluster);
    }

    std::sort(scored_clusters.begin(), scored_clusters.end(),
              [](const ScoredCluster& a, const ScoredCluster& b) {
                  return a.first < b.first;
              });

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> best_clusters;
    for (int i = 0; i < std::min(n, int(scored_clusters.size())); ++i) {
        best_clusters.push_back(scored_clusters[i].second);
    }

    return best_clusters;
}
//utilize ransac to remove noise points
std::tuple<Eigen::Vector3f, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> ransacCircleFit(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_points,
                                                                                        const Eigen::Vector4f &plane_coeffs,
                                                                                        int ransac_iterations ,
                                                                                        float distance_threshold ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr max_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f best_circle(0, 0, 0);
    float best_radius = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, cluster_points->points.size() - 1);

    for (int i = 0; i < ransac_iterations; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr sample_points(new pcl::PointCloud<pcl::PointXYZ>);
        std::set<int> sample_indices;

        while (sample_indices.size() < 5) {
            sample_indices.insert(dis(gen)); 
        }

        for (const auto &index : sample_indices) {
            sample_points->points.push_back(cluster_points->points[index]);
        }

        auto [circle_candidate, radius_candidate] = circleFitConstrained2D(sample_points, plane_coeffs);

        pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &point : cluster_points->points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            float distance = (p.head<2>() - circle_candidate.head<2>()).norm();
            if (std::abs(distance - radius_candidate) < distance_threshold) {
                inliers->points.push_back(point);
            }
        }

        if (inliers->points.size() > max_inliers->points.size()) {
            max_inliers = inliers;
            best_circle = circle_candidate;
            best_radius = radius_candidate;
        }
    }

    return {best_circle, best_radius, max_inliers};
}
std::pair<Eigen::Vector3f, Eigen::Vector3f> findAxes(const Eigen::Vector4f &plane_coeffs) {
    float a = plane_coeffs[0], b = plane_coeffs[1], c = plane_coeffs[2];
    Eigen::Vector3f u1, u2;

    if (a != 0 || b != 0) {
        u1 = Eigen::Vector3f(-b, a, 0);
    } else {
        u1 = Eigen::Vector3f(1, 0, 0);
    }

    u2 = Eigen::Vector3f(a, b, c).cross(u1);

    u1.normalize();
    u2.normalize();

    return {u1, u2};
}
//find circle center per one circle cluster
std::pair<Eigen::Vector3f, float> circleFitConstrained2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &points, const Eigen::Vector4f &plane_coeffs) {
    auto [u1, u2] = findAxes(plane_coeffs);
    Eigen::Vector3f center_point(0, 0, 0);

    for (const auto &point : points->points) {
        center_point += point.getVector3fMap();
    }
    center_point /= points->points.size();

    Eigen::Matrix4f Rot;
    Rot << u1[0], u2[0], plane_coeffs[0], center_point[0],
           u1[1], u2[1], plane_coeffs[1], center_point[1],
           u1[2], u2[2], plane_coeffs[2], center_point[2],
           0, 0, 0, 1;

    Eigen::Matrix4f R_inv = Rot.inverse();
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_2d(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &point : points->points) {
        Eigen::Vector4f point_homogeneous(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f point_2d_homogeneous = R_inv * point_homogeneous;
        points_2d->points.emplace_back(point_2d_homogeneous[0], point_2d_homogeneous[1], 0.0f);
    }

    Eigen::MatrixXf A(points_2d->points.size(), 3);
    Eigen::VectorXf B(points_2d->points.size());

    for (size_t i = 0; i < points_2d->points.size(); ++i) {
        A(i, 0) = 2 * points_2d->points[i].x;
        A(i, 1) = 2 * points_2d->points[i].y;
        A(i, 2) = 1;
        B(i) = points_2d->points[i].x * points_2d->points[i].x + points_2d->points[i].y * points_2d->points[i].y;
    }

    Eigen::Vector3f circle_params = A.colPivHouseholderQr().solve(B);
    float x0 = circle_params[0], y0 = circle_params[1];
    float radius = std::sqrt(x0 * x0 + y0 * y0 + circle_params[2]);

    Eigen::Vector4f circle_center_2d(x0, y0, 0, 1);
    Eigen::Vector4f circle_center_3d_homogeneous = Rot * circle_center_2d;
    Eigen::Vector3f circle_center_3d(circle_center_3d_homogeneous[0], circle_center_3d_homogeneous[1], circle_center_3d_homogeneous[2]);

    return {circle_center_3d, radius};
}
//sort centers -> 3d to 2d and apply camera version sorting 
std::vector<Eigen::Vector3f> sortCentersByRotatedLidarFrame(const std::vector<Eigen::Vector3f>& circle_centers, int nx, int ny) {

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& pt : circle_centers)
        mean += pt;
    mean /= circle_centers.size();

    Eigen::Vector2f direction_2d = mean.head<2>().normalized();  
    float theta = std::atan2(direction_2d.y(), direction_2d.x());  // 

    Eigen::Matrix3f R;
    R << std::cos(-theta), -std::sin(-theta), 0,
         std::sin(-theta),  std::cos(-theta), 0,
         0,                 0,                1;
    
    std::vector<cv::Point2f> yz_coords;
    std::vector<std::pair<cv::Point2f, float>> yz_with_x;
    for (size_t i = 0; i < circle_centers.size(); ++i) {
        const auto& pt = circle_centers[i];
        Eigen::Vector3f pt_rot = R * pt;
        cv::Point2f yz(pt_rot.y(), pt_rot.z());
        yz_coords.emplace_back(yz);
        yz_with_x.emplace_back(yz, pt_rot.x());
    }

    CircleGridFinder gridfinder(false);
    std::vector<cv::Point2f> dist(nx * ny);
    gridfinder.findGrid(yz_coords, cv::Size(nx, ny), dist);
    if (nx == ny && dist.front().y < dist.back().y) {
        std::vector<cv::Point2f> new_dist(nx * ny);
        for (int i = 0; i < nx * ny; ++i) {
            int row = ny - 1 - (i % ny);
            int col = i / ny;
            int new_idx = row * nx + col;
            new_dist[new_idx] = dist[i];
        }
        dist = new_dist;
    }
    Eigen::Matrix3f R_inv = R.transpose();
    std::vector<Eigen::Vector3f> sorted_centers;
    for (const auto& pt2d : dist) {
        auto it = std::find_if(yz_with_x.begin(), yz_with_x.end(),
            [&pt2d](const std::pair<cv::Point2f, float>& p) {
                return p.first == pt2d;
            });

        if (it != yz_with_x.end()) {
            Eigen::Vector3f pt_rot(it->second, pt2d.x, pt2d.y);  // (x, y, z) in rotated frame
            Eigen::Vector3f pt_orig = R_inv * pt_rot;            // transform back to original frame
            sorted_centers.emplace_back(pt_orig);
        }
    }
    

    return sorted_centers;
}
//find se3 using ceres slover
Eigen::MatrixXf board_points;
Eigen::Matrix4f optimize_rotation_transform_general(
    const std::vector<Eigen::Vector3f> &cluster_centers,
    const Eigen::Vector4f &plane_coeffs,
    int n_x, int n_y,
    float distance)
{
    int base_idx = (n_y - 1) * n_x;
    Eigen::Vector3f center_ref = cluster_centers[base_idx];
    Eigen::Vector3f center_next = cluster_centers[base_idx + 1];

    float a = plane_coeffs(0), b = plane_coeffs(1), c = plane_coeffs(2), d = plane_coeffs(3);
    if (d < 0) { a = -a; b = -b; c = -c; d = -d; }

    Eigen::Vector3f w = center_next - center_ref;
    w.normalize();
    Eigen::Vector3f v = Eigen::Vector3f(a, b, c).cross(w);
    Eigen::Matrix4f R = compute_R(w, v, a, b, c, center_ref);
    Eigen::Matrix4f R_inv = R.inverse();

    Eigen::MatrixXf world_points(4, cluster_centers.size());
    for (size_t i = 0; i < cluster_centers.size(); ++i)
        world_points.col(i) << cluster_centers[i], 1.0f;

    Eigen::MatrixXf transformed_world_points = (R_inv * world_points).topRows(3).eval();

    double theta_init = 0.0;
    double x_offset_init = -distance;
    double y_offset_init = -distance;

    int num_points = n_x * n_y;
    board_points.resize(3, num_points);

    for (int j = 0; j < n_y; ++j) {
        for (int i = 0; i < n_x; ++i) {
            int idx = j * n_x + i;
            board_points(0, idx) = distance * (i + 1);
            board_points(1, idx) = distance * (n_y - j);
            board_points(2, idx) = 1.0f;
        }
    }

    Residuals* residuals = new Residuals(R_inv, transformed_world_points, num_points);
    ceres::Problem problem;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Residuals, ceres::DYNAMIC, 1, 1, 1>(
            residuals, 2 * num_points),
        nullptr,
        &theta_init,
        &x_offset_init,
        &y_offset_init
    );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Matrix4f optimized_R_board;
    optimized_R_board << std::cos(theta_init), -std::sin(theta_init), 0.0f, x_offset_init,
                         std::sin(theta_init),  std::cos(theta_init), 0.0f, y_offset_init,
                         0.0f, 0.0f, 1.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 1.0f;

    Eigen::Matrix4f final_transform = R * optimized_R_board;

    return final_transform;
}
Eigen::Matrix4f compute_R(Eigen::Vector3f w, Eigen::Vector3f v, float a, float b, float c, Eigen::Vector3f center_5) {
    Eigen::Matrix4f R;
    R << w(0), v(0), a, center_5(0),
         w(1), v(1), b, center_5(1),
         w(2), v(2), c, center_5(2),
         0, 0, 0, 1;
    return R;
}
