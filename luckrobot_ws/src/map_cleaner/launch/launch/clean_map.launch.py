from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_cleaner',
            executable='map_cleaner_node',
            name='map_cleaner',
            output='screen',
            parameters=[{
                # 原始 PCD 路径
                'input_pcd': '/home/nvidia/luckrobot/mid360s_ws/map/test.pcd',
                # 清洗后保存路径
                'output_pcd': '/home/nvidia/luckrobot/mid360s_ws/map/test_cleaned.pcd',
                
                # 降采样分辨率（0.05米意味着每5厘米保存一个点，地图会很平滑且体积极小）
                'voxel_size': 0.05,
                
                # 去噪强度（如果地图里还有很多散点，可以把 sor_stddev 调小，如 0.8）
                'sor_mean_k': 20,
                'sor_stddev': 1.0,
            }]
        )
    ])