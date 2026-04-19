import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ==========================================
    # 阶段 1：底层硬件与传感器 (事件级错峰启动)
    # ==========================================
    
    # [事件 1] T=0s，立即拉起底盘控制
    wheel_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('luckrobot_launch1'), 'launch', 'wheel_control_launch.py'])
        )
    )

    # [事件 2] T=2s，延时错峰拉起 Robot Display
    robot_display_event = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="============ [事件 2] 2秒错峰：拉起 Robot Display ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('robot_display'), 'launch', 'robot_display.launch.py'])
                )
            )
        ]
    )

    # [事件 3] T=4s，延时错峰拉起 Livox 雷达
    livox_driver_event = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="============ [事件 3] 4秒错峰：拉起 Livox 雷达驱动 ============"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py'])
                )
            )
        ]
    )

    # ==========================================
    # 阶段 2：发布移动指令，并挂载"物理动作监听器"
    # ==========================================
    
    # 丝杠话题发布命令 (瞬间发送完毕)
    pub_lead_screw_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/lead_screw/displacement', 
            'std_msgs/msg/Float32', '{data: -462.0}', '--once'
        ],
        output='screen'
    )

    # 🔥 【神级事件锚点】：物理位置监听器
    # 使用 Linux 管道命令监听 ROS 全局日志。一旦监听到丝杠节点打出包含 "-462.0" 的日志，
    # grep -m 1 就会满足条件并强行退出该进程！
    wait_motor_finish_cmd = ExecuteProcess(
        cmd=['ros2 topic echo /rosout | grep -m 1 -- "-462.0"'],
        shell=True,
        output='log' # 隐藏监听日志，不刷屏
    )

    # 雷达在第 4 秒启动，要求相隔 25 秒，所以在第 29 秒触发下发
    pub_event = TimerAction(
        period=29.0,
        actions=[
            LogInfo(msg="============ [事件 4] 距离雷达启动已过 25s：下发下降指令，并开始监听物理结束信号 ============"),
            pub_lead_screw_cmd,
            wait_motor_finish_cmd # 同时启动监听特工
        ]
    )

    # ==========================================
    # 阶段 3 & 4：基于物理移动结束事件，拉起高算力算法
    # ==========================================
    
    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('open3d_loc'), 'launch', 'localization_3d_g1.launch.py'])
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_luckrobot'), 'launch', 'nav2.launch.py'])
        )
    )

    # 🔥 核心事件钩子：不再监听发送指令，而是死死盯住 wait_motor_finish_cmd 监听进程！
    open3d_and_nav2_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_motor_finish_cmd,  # 只有丝杠彻底走完并打印日志，这个 target 才会 Exit
            on_exit=[
                # 【事件 5】：一旦确认物理移动彻底结束，瞬间全功率拉起 Open3D
                LogInfo(msg="============ [事件 5] 接收到物理移动彻底完成信号！瞬间拉起 Open3D ============"),
                open3d_loc_launch,
                
                # 【事件 6】：在拉起 Open3D 的同时，开启 20 秒资源缓冲倒计时
                LogInfo(msg="============ [等待中] 已开启 20 秒硬件资源缓冲倒计时 ============"),
                TimerAction(
                    period=20.0,
                    actions=[
                        LogInfo(msg="============ [事件 6] 20 秒缓冲期结束！全功率拉起 Nav2 导航栈 ============"),
                        nav2_launch
                    ]
                )
            ]
        )
    )

    # ==========================================
    # 组装返回：注册所有的事件监听器与动作
    # ==========================================
    return LaunchDescription([
        LogInfo(msg="============ [系统启动] 物理级严格流水线事件引擎已启动 ============"),
        LogInfo(msg="============ [事件 1] 立即触发：拉起 Wheel Control ============"),
        wheel_control_launch,
        robot_display_event,
        livox_driver_event,
        pub_event,
        open3d_and_nav2_event_handler # 注册底层物理事件监听器
    ])