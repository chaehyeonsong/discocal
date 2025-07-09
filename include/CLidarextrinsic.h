#ifndef __CLIDAR_EXTRINSIC_H__
#define __CLIDAR_EXTRINSIC_H__

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "utils.h"  // se3 ���� ����
#include "CLieAlgebra.h"
#include "Lidarparam.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <limits>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/filters/extract_indices.h>
#include <random> 
#include "CCircleGridFinder.hpp"
#include <ceres/ceres.h>
#include <pcl/visualization/pcl_visualizer.h> 

#include <thread>
#include <chrono>
#include <future>
#include <sys/wait.h> 
#include <unistd.h>   
struct LidarCalibrationParams;



bool estimate_lidar_extrinsic(const std::string& pcd_path,
                               const LidarCalibrationParams& params,
                               se3& out_pose, int cnt);
void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *);
bool visualizeCloudInteractive(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& title);
bool visualizeClustersInteractive(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, const std::string& title) ;
void applyBoundaryFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         const LidarCalibrationParams& params,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);

std::vector<pcl::PointIndices> clusterPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                              float eps = 0.05, int min_samples = 10);

pcl::PointCloud<pcl::PointXYZ>::Ptr findMostPlanarCluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices); 

std::pair<Eigen::Vector4f, pcl::PointIndices::Ptr> detectPlanes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double distance_threshold = 0.025, int ransac_n = 3, int max_iterations = 10000);

Eigen::Vector4f pcaPlaneFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr projectPointsToPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector4f& plane_model);

pcl::PointCloud<pcl::PointXYZ>::Ptr findBoundaryPointsImproved(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double radius = 0.02, double min_distance = 0.000, double cdr_threshold = 0.42, double direction_var_threshold = 1.0);

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
selectNCircleClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      double distance, double radius, int n);

std::tuple<Eigen::Vector3f, float, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
ransacCircleFit(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_points,
                const Eigen::Vector4f &plane_coeffs,
                int ransac_iterations = 3000,
                float distance_threshold = 0.01);
std::pair<Eigen::Vector3f, Eigen::Vector3f> findAxes(const Eigen::Vector4f &plane_coeffs);
std::pair<Eigen::Vector3f, float> 
circleFitConstrained2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr &points, const Eigen::Vector4f &plane_coeffs);
Eigen::Matrix4f optimize_rotation_transform_general(
    const std::vector<Eigen::Vector3f> &cluster_centers,
    const Eigen::Vector4f &plane_coeffs,
    int n_x, int n_y,
    float distance);
Eigen::Matrix4f compute_R(Eigen::Vector3f w, Eigen::Vector3f v, float a, float b, float c, Eigen::Vector3f center_5);
std::vector<Eigen::Vector3f> sortCentersByRotatedLidarFrame(const std::vector<Eigen::Vector3f>& circle_centers, int nx, int ny);
extern Eigen::MatrixXf board_points;
struct Residuals {
    Residuals(Eigen::Matrix4f R_inv, Eigen::MatrixXf transformed_world_points, int num_points)
        : R_inv(R_inv), transformed_world_points(transformed_world_points), num_points(num_points) {}

    template <typename T>
    bool operator()(const T* const theta, const T* const x_offset, const T* const y_offset, T* residual) const {
        T cos_theta = ceres::cos(theta[0]);
        T sin_theta = ceres::sin(theta[0]);

        Eigen::Matrix<T, 3, 3> R_board;
        R_board << cos_theta, -sin_theta, x_offset[0],
                   sin_theta,  cos_theta, y_offset[0],
                   T(0), T(0), T(1);

        Eigen::Matrix<T, 3, Eigen::Dynamic> transformed_points = R_board.template cast<T>() * board_points.template cast<T>();
        Eigen::Matrix<T, 2, Eigen::Dynamic> transformed_points_2d = transformed_points.topRows(2);
        Eigen::Matrix<T, 2, Eigen::Dynamic> transformed_world_points_2d = transformed_world_points.topRows(2).template cast<T>();

        Eigen::Matrix<T, 2, Eigen::Dynamic> diff = transformed_points_2d - transformed_world_points_2d;

        for (int i = 0; i < num_points; ++i) {
            residual[2 * i] = diff(0, i);
            residual[2 * i + 1] = diff(1, i);
        }

        return true;
    }

    Eigen::Matrix4f R_inv;
    Eigen::MatrixXf transformed_world_points;
    int num_points;
};
#endif