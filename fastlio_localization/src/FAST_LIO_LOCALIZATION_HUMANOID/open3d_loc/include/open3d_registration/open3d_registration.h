#pragma once

#include <iostream>

#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// using namespace open3d;

namespace pcd_tools
{

    open3d::pipelines::registration::RegistrationResult RegistrationFpfh(
        std::shared_ptr<open3d::geometry::PointCloud> source,
        std::shared_ptr<open3d::geometry::PointCloud> target,
        std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh,
        std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh,
        float voxel_size,
        open3d::utility::optional<unsigned int> seed_ = 123456,
        bool mutual_filter = true);

    /// @brief
    /// @param source
    /// @param target
    /// @param icp_distance_threshold
    /// @param init_matrix initial pose for source pcd
    /// @param icp_method 0: point2point 1: point2plane 2: generalizedIcp
    /// @param icp_iteration icp iteration times
    /// @return
    open3d::pipelines::registration::RegistrationResult RegistrationIcp(
        std::shared_ptr<open3d::geometry::PointCloud> source,
        std::shared_ptr<open3d::geometry::PointCloud> target,
        float icp_distance_threshold,
        Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity(),
        int icp_method = 1, int icp_iteration = 30);

    /// @brief use diffirent voxel_size(diffirent threshold) to registration
    /// @param source
    /// @param target
    /// @param voxel_size
    /// @param icp_method 0: point2point 1: point2plane 2: generalizedIcp
    /// @param multiscale use multi scale for icp
    /// @return
    Eigen::Matrix4d RegistrationMultiScaleIcp(std::shared_ptr<open3d::geometry::PointCloud> source,
                                              std::shared_ptr<open3d::geometry::PointCloud> target,
                                              double voxel_size, int icp_method = 1, std::vector<double> scale = {1.5});

    open3d::pipelines::registration::RegistrationResult RegistrationEvaluate(const std::shared_ptr<open3d::geometry::PointCloud> src, const std::shared_ptr<open3d::geometry::PointCloud> tgt, double voxel_size, Eigen::Matrix4d transformation);

    /// @brief Implementation of point cloud registration pipeline
    class Open3dRegistration
    {
    private:
        /* data */
    public:
        Open3dRegistration(/* args */);
        Open3dRegistration(const Open3dRegistration &o3d_reg);
        ~Open3dRegistration();

        /// @brief reset parameters
        void ResetParam();

        /// @brief
        /// @param pcd pointcloud need to be process
        /// @param voxel_size_ downsample voxel size
        /// @param cropbox if extent.x != 0, use crop
        /// @return downsampled pcd with normal, fpfh features
        std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
                   std::shared_ptr<open3d::pipelines::registration::Feature>>
        PreprocessPointCloud(
            std::shared_ptr<open3d::geometry::PointCloud> pcd,
            float voxel_size_ = 2.0,
            bool filter = true,
            open3d::geometry::OrientedBoundingBox cropbox = open3d::geometry::OrientedBoundingBox());

        /// @brief implementation of registration pipeline
        /// @return is registration failed?
        bool RegistrationPipeline();

        Eigen::Matrix4d GetFinalMatrix();

        /// @brief print current parameters for regiatration
        void PrintParameters();
        /// @brief 打印配准结果
        void PrintRegistrationResult(const open3d::pipelines::registration::RegistrationResult &result);

    public:
        bool voxel_downsample = true;
        bool use_fpfh = true;
        bool use_icp = true;
        bool statistical_filter_source = true;
        bool statistical_filter_target = true;
        int neighbors = 50;
        double std_ratio = 3.0;

        std::string path_source = "";
        std::string path_target = "";
        // set voxel_size
        float voxel_size = 0.05;

        // float distance_threshold = voxel_size * 1.5; // fpfh distance_threshold
        // use icp to fine tuning

        int icp_iteration = 30;

        // float icp_distance_threshold = voxel_size * 1.5;

        // icp_method(optional): point to point(0), point to plane(1), generalized icp(2), default 1
        int icp_method = 1;

        // reduce icp distance threshold to 80%,
        // use last icp result and new icp distance threshold to continue finetune
        // int icp_optimation_times = 1;
        // double reduce_ratio = 0.8;

        // fix seed for ransac to get a deterministic registration result(rt), set 0 to get a variant rt-->
        open3d::utility::optional<unsigned int> seed_ = open3d::utility::nullopt;
        // utility::optional<unsigned int> seed_ = 123456;
        bool mutual_filter = true;

        /// @brief input initial_matrix
        Eigen::Matrix4d initial_matrix = Eigen::Matrix4d::Identity();
        /// @brief fpfh+ransac registration result
        Eigen::Matrix4d fpfh_matrix = Eigen::Matrix4d::Identity();
        /// @brief icp registration matrix
        Eigen::Matrix4d icp_matrix = Eigen::Matrix4d::Identity();
        /// @brief final_transformation = icp_matrix * fpfh_matrix * initial_matrix
        Eigen::Matrix4d final_transformation = Eigen::Matrix4d::Identity();

        /// @brief pcd need to registration
        std::shared_ptr<open3d::geometry::PointCloud> source, target;
        std::shared_ptr<open3d::geometry::PointCloud> source_ori, target_ori;
        std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh, target_fpfh;

        /// @brief cropbox for pcd
        std::shared_ptr<open3d::geometry::OrientedBoundingBox> CropBox_source;
        std::shared_ptr<open3d::geometry::OrientedBoundingBox> CropBox_target;

        /// @brief registration result
        open3d::pipelines::registration::RegistrationResult regresult;
        /// @brief overlap between source and target after registration
        double overlap = 0.0;
        /// @brief 配准overlap阈值，小于该阈值则认为配准失败
        double threshold_overlap = 0.8;
    };
}