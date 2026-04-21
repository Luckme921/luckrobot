from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    open3d_loc_share = FindPackageShare('open3d_loc')

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 配置文件路径
    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_g1.yaml'
    ])

    # 地图文件路径 - 使用绝对路径指向源码目录中的地图文件
    map_file = '/home/nvidia/luckrobot/mid360s_ws/map/test.pcd'

    # 静态TF发布节点 - camera_init to odom
    static_tf_camera_init2odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']
    )

    # 静态TF发布节点 - imu_link to base_link
    # 修正：父frame是imu_link，子frame是base_link
    # static_tf_imulink2baselink = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='imulink2baselink',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'imu_link', 'base_link']
    # )

    # 静态TF发布节点 - base_link to motion_link
    # 修正：base_link是父frame，motion_link是子frame
    static_tf_base_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_center_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0',
                   '1', 'base_link', 'motion_link']
    )

    # 全局定位节点
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        # ====== 重映射 ======
        remappings=[
            ('/map', '/map_3d'),
            ('/scan', '/scan_3d') 
        ],
        # ===================
        parameters=[
            config_file,
            {
                'path_map': map_file,
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.01,
                'voxelsize_fine': 0.2,
                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,
                'loc_frequence': 2.5,
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # 点云转换节点
    pointcloud_transformer_node = Node(
        package='open3d_loc',
        executable='pointcloud_transformer_node',
        name='pointcloud_transformer_node',
        output='screen',
        parameters=[{
            'input_topic': '/cloud_registered_body_1',
            'output_topic': '/cloud_registered_map',
            'global_map_topic': '/global_map',
            'source_frame': 'base_link',
            'target_frame': 'map',
            'voxel_leaf_size': 0.1,
            'map_voxel_leaf_size': 0.2,
            'max_global_points': 1000000,
            'map_publish_frequency': 1.0,
            'enable_global_map': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_registered_body_1'),  # 订阅 FAST-LIO 吐出的实时畸变校正点云
            ('scan', '/scan_2d')                       # 避开原本占用的 /scan，输出全新的 2D 话题给 Nav2
        ],
        parameters=[{
            'target_frame': 'body', # 统一投影到地面参考系 (如果TF树报错找不到它，可暂时改为 'body' 或 'base_link' 并调整高度)
            'transform_tolerance': 0.01,
            'min_height': -0.4,                # 最低高度：0.5米（过滤掉地面和轻微的斜坡干扰）
            'max_height': 0.2,               # 最高高度：0.95米（正好在你雷达 0.9m 的下方，避免切到雷达支架）
            'angle_min': -3.14159,            # -180度
            'angle_max': 3.14159,             # 180度
            'angle_increment': 0.0087,        # 角度分辨率 (约0.5度)
            'scan_time': 0.1,                 # 10Hz
            'range_min': 0.3,                 # 过滤掉雷达近处的车体反光盲区
            'range_max': 20.0,                # 最大探测距离
            'use_inf': True,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_camera_init2odom,
        # static_tf_imulink2baselink,
        static_tf_base_center,
        global_localization_node,
        # pointcloud_transformer_node
        pointcloud_to_laserscan_node
    ])
