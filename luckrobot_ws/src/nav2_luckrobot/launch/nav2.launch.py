import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取目录
    nav2_luckrobot_dir = get_package_share_directory('nav2_luckrobot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_config_dir = os.path.join(nav2_luckrobot_dir, 'rviz', 'my_nav2_view.rviz')
    # 强制实车环境使用 False
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    
    # 读取你通过 pcd2pgm 保存的干净的 2D 栅格地图
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(nav2_luckrobot_dir, 'maps', 'test_map.yaml'))
    
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(nav2_luckrobot_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # =========================================================
        # 1. 单独启动 Map Server (提供全局 2D 代价底图)
        # =========================================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # =========================================================
        # 2. 启动核心导航层 (完全绕过 localization_launch 和 AMCL)
        # =========================================================
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),

        # =========================================================
        # 3. 启动 RViz2
        # =========================================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])