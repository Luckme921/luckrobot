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
                'input_pcd': '/home/nvidia/luckrobot/mid360s_ws/map/home.pcd',
                # 清洗后保存路径
                'output_pcd': '/home/nvidia/luckrobot/mid360s_ws/map/home_cleaned.pcd',
                
            }]
        )
    ])