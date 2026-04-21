from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument # <--- 新增导入 DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration # <--- 新增导入 LaunchConfiguration
from launch.conditions import IfCondition # <--- 新增导入 IfCondition
import os


def generate_launch_description():
    # ==========================================
    # 1. 声明一个可以通过命令行传入的参数 'use_rviz'
    # ==========================================
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',  # 默认值为 false（默认打开 RViz）
        description='Whether to start RViz2'
    )
    # 获取这个参数的当前值
    use_rviz = LaunchConfiguration('use_rviz')

    # 获取包路径
    fast_lio_share = FindPackageShare('fast_lio')
    open3d_loc_share = FindPackageShare('open3d_loc')

    # 包含 fast_lio 的 launch 文件
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fast_lio_share,
                'launch',
                'mapping.launch.py'
            ])
        ])
    )

    # 包含 open3d_loc 的 launch 文件
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'open3d_loc_g1.launch.py'
            ])
        ])
    )

    # RViz 节点配置
    rviz_config_path = PathJoinSubstitution([
        open3d_loc_share,
        'rviz_cfg',
        'fastlio.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        arguments=['-d', rviz_config_path],
        output='screen',
        prefix='nice',
        # ==========================================
        # 2. 给 RViz 节点加上条件判断！
        # ==========================================
        condition=IfCondition(use_rviz)  # 只有当 use_rviz 为 'true' 时，才启动这个节点
    )

    return LaunchDescription([
        use_rviz_arg,      # <--- 3. 别忘了把声明的参数扔进 LaunchDescription 列表里
        fast_lio_launch,
        open3d_loc_launch,
        rviz_node
    ])