#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZI PointType;

class MapCleanerNode : public rclcpp::Node
{
public:
    MapCleanerNode() : Node("map_cleaner_node")
    {
        // ===================================================================================
        // 核心调参区 (当前保持与你上次运行一致的参数，请仔细阅读注释后进行调节)
        // ===================================================================================

        this->declare_parameter<std::string>("input_pcd", "/home/nvidia/luckrobot/mid360s_ws/map/home.pcd");
        this->declare_parameter<std::string>("output_pcd", "/home/nvidia/luckrobot/mid360s_ws/map/home_cleaned.pcd");
        
        // 1. 体素滤波 (Voxel Grid) —— 作用：降采样，让地图均匀稀疏化
        // 【调节建议】：一般保持 0.05 (5厘米) 不变。如果太大，地图会糊掉；太小，计算量爆炸。
        this->declare_parameter<double>("voxel_size", 0.04); 

        // 2. 统计滤波 (SOR) —— 💡 解决你当前问题的关键点 1 (打断连接)
        // 作用：寻找离散点。原理是看每个点周围 k 个点，距离是不是太远。
        this->declare_parameter<int>("sor_mean_k", 20);      // 参考的邻居数量。20~50 都可以，不用怎么动。
        // 【重点调节】：标准差倍数。值越小，过滤越严格，越能“剃掉”边缘的稀疏连接点。
        // 【当前 1.0】如果噪声簇和墙壁还有点连着，尝试降到【0.5】，再不行降到【0.2】。
        this->declare_parameter<double>("sor_stddev", 0.2);  

        // 3. 欧式聚类 (Euclidean Clustering) —— 💡 解决你当前问题的关键点 2 (丢弃大块悬浮物)
        // 作用：把距离相近的点“抱团”。
        // 【重点调节】：聚类搜索半径(米)。距离小于这个值的点会被判定为同一个物体。
        // 【当前 0.2】如果悬浮物离墙壁有十几二十厘米，0.2 就太大了，会把它们连城一片。
        // 尝试降到【0.15】或【0.1】。注意：不能小于 voxel_size 的两倍，否则好好的墙也会被切碎。
        this->declare_parameter<double>("cluster_tolerance", 0.08); 
        
        // 【重点调节】：最小有效簇的点数。点数低于这个值的“团伙”会被当做垃圾直接删掉。
        // 【当前 500】你的主地图有将近 12 万个点，完全可以把这个值放心大胆地拉高到【1000】或【5000】。
        this->declare_parameter<int>("min_cluster_size", 200);     
        
        this->declare_parameter<int>("max_cluster_size", 10000000); // 簇的最大点数 (设为极大值以保留主地图，不用改)

        // ===================================================================================

        // 获取参数
        std::string input_file = this->get_parameter("input_pcd").as_string();
        std::string output_file = this->get_parameter("output_pcd").as_string();
        double voxel_size = this->get_parameter("voxel_size").as_double();
        int sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        double sor_stddev = this->get_parameter("sor_stddev").as_double();
        double cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
        int min_cluster_size = this->get_parameter("min_cluster_size").as_int();
        int max_cluster_size = this->get_parameter("max_cluster_size").as_int();

        RCLCPP_INFO(this->get_logger(), "开始清洗地图（包含聚类去噪）...");
        RCLCPP_INFO(this->get_logger(), "输入文件: %s", input_file.c_str());
        RCLCPP_INFO(this->get_logger(), "参数设定 -> SOR StdDev: %.2f | 聚类半径: %.2f | 最小聚类点数: %d", 
                    sor_stddev, cluster_tolerance, min_cluster_size);

        // 执行点云处理
        processMap(input_file, output_file, voxel_size, sor_mean_k, sor_stddev, 
                   cluster_tolerance, min_cluster_size, max_cluster_size);
    }

private:
    void processMap(const std::string& input_file, const std::string& output_file, 
                    double voxel_size, int sor_mean_k, double sor_stddev,
                    double cluster_tolerance, int min_cluster_size, int max_cluster_size)
    {
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered_voxel(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered_sor(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered_cluster(new pcl::PointCloud<PointType>);

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

        // B. 统计滤波 (精细去噪，打断边缘飞点连接)
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(cloud_filtered_voxel);
        sor.setMeanK(sor_mean_k);
        sor.setStddevMulThresh(sor_stddev);
        sor.filter(*cloud_filtered_sor);
        RCLCPP_INFO(this->get_logger(), "统计去噪后点数: %zu", cloud_filtered_sor->points.size());

        // C. 欧式聚类提取 (去除空间中悬浮的独立点云簇)
        RCLCPP_INFO(this->get_logger(), "正在进行欧式聚类分析...");
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
        tree->setInputCloud(cloud_filtered_sor);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(cluster_tolerance); 
        ec.setMinClusterSize(min_cluster_size);    
        ec.setMaxClusterSize(max_cluster_size);    
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered_sor);
        ec.extract(cluster_indices);

        // 将所有合格的聚类合并为最终地图
        for (const auto& cluster : cluster_indices)
        {
            for (const auto& idx : cluster.indices)
            {
                cloud_filtered_cluster->points.push_back(cloud_filtered_sor->points[idx]);
            }
        }
        cloud_filtered_cluster->width = cloud_filtered_cluster->points.size();
        cloud_filtered_cluster->height = 1;
        cloud_filtered_cluster->is_dense = true;

        RCLCPP_INFO(this->get_logger(), "聚类去噪后最终点数: %zu (共保留了 %zu 个主要结构)", 
                    cloud_filtered_cluster->points.size(), cluster_indices.size());

        // D. 保存地图
        if (pcl::io::savePCDFileBinary(output_file, *cloud_filtered_cluster) == -1) {
            RCLCPP_ERROR(this->get_logger(), "保存失败！");
        } else {
            RCLCPP_INFO(this->get_logger(), "清洗成功！文件已存至: %s", output_file.c_str());
        }
        
        // 注意：这里删除了引发报错的 rclcpp::shutdown()
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 初始化节点，构造函数内会同步执行建图逻辑
    auto node = std::make_shared<MapCleanerNode>();
    
    // 因为这是一个处理静态文件的脚本程序，处理完成后不需要像持续监听传感器那样自旋(spin)
    // 所以这里直接结束并清理 ROS 环境，完美解决原本的生命周期报错
    rclcpp::shutdown();
    return 0;
}