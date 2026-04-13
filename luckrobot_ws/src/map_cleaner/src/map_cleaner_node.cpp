#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZI PointType;

class MapCleanerNode : public rclcpp::Node
{
public:
    MapCleanerNode() : Node("map_cleaner_node")
    {
        // 1. 声明参数
        this->declare_parameter<std::string>("input_pcd", "/home/nvidia/luckrobot/mid360s_ws/map/test.pcd");
        this->declare_parameter<std::string>("output_pcd", "/home/nvidia/luckrobot/mid360s_ws/map/test_cleaned.pcd");
        this->declare_parameter<double>("voxel_size", 0.05); // 体素滤波分辨率
        this->declare_parameter<int>("sor_mean_k", 20);      // SOR 邻域点数
        this->declare_parameter<double>("sor_stddev", 1.0);  // SOR 标准差阈值（越小越严格）

        // 2. 获取参数
        std::string input_file = this->get_parameter("input_pcd").as_string();
        std::string output_file = this->get_parameter("output_pcd").as_string();
        double voxel_size = this->get_parameter("voxel_size").as_double();
        int sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        double sor_stddev = this->get_parameter("sor_stddev").as_double();

        RCLCPP_INFO(this->get_logger(), "开始清洗地图（保留完整3D结构）...");
        RCLCPP_INFO(this->get_logger(), "输入文件: %s", input_file.c_str());
        RCLCPP_INFO(this->get_logger(), "输出文件: %s", output_file.c_str());

        // 3. 执行点云处理
        processMap(input_file, output_file, voxel_size, sor_mean_k, sor_stddev);
    }

private:
    void processMap(const std::string& input_file, const std::string& output_file, 
                    double voxel_size, int sor_mean_k, double sor_stddev)
    {
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered_voxel(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered_sor(new pcl::PointCloud<PointType>);

        // 加载原始地图
        if (pcl::io::loadPCDFile<PointType>(input_file, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法读取文件: %s", input_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "原始点数: %zu", cloud->points.size());

        // A. 体素滤波 (降采样/均匀化)
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*cloud_filtered_voxel);
        RCLCPP_INFO(this->get_logger(), "体素滤波后点数: %zu", cloud_filtered_voxel->points.size());

        // B. 统计滤波 (精细去噪)
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(cloud_filtered_voxel);
        sor.setMeanK(sor_mean_k);
        sor.setStddevMulThresh(sor_stddev);
        sor.filter(*cloud_filtered_sor);
        RCLCPP_INFO(this->get_logger(), "统计去噪后最终点数: %zu", cloud_filtered_sor->points.size());

        // C. 直接保存 (保留完整高度，不切片)
        if (pcl::io::savePCDFileBinary(output_file, *cloud_filtered_sor) == -1) {
            RCLCPP_ERROR(this->get_logger(), "保存失败！");
        } else {
            RCLCPP_INFO(this->get_logger(), "清洗成功！文件已存至: %s", output_file.c_str());
        }
        
        // 执行完自动关闭
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapCleanerNode>();
    rclcpp::spin(node);
    return 0;
}