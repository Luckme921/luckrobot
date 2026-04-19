#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler

def generate_launch_description():
    target_pkg = "wheel_controller"

    # ==================== 1. 丝杠电机节点（最先启动） ====================
    lead_screw_motor_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="lead_screw_motor_node",
        name="lead_screw_motor_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 2. 底盘控制节点（丝杠节点启动后执行） ====================
    wheel_controller_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="wheel_controller_node",
        name="wheel_controller_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 3. 危险指令节点（底盘节点启动后执行） ====================
    danger_command_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="danger_command_node",
        name="danger_command_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    state_publisher_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="state_publisher_node",
        name="state_publisher_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 事件动作：严格顺序启动 ====================
    # 事件1：丝杠节点启动后 → 启动底盘控制节点
    event_start_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lead_screw_motor_node,  # 监听丝杠节点启动
            on_start=[wheel_controller_node]       # 启动后执行的动作
        )
    )

    # 事件2：底盘节点启动后 → 启动危险指令节点
    event_start_danger_command = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=wheel_controller_node,  # 监听底盘节点启动
            on_start=[danger_command_node]        # 启动后执行的动作
        )
    )

    event_start_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=danger_command_node,  # 监听危险指令节点启动
            on_start=[state_publisher_node]      # 启动后执行的动作
        )
    )

    # ==================== 组装启动描述 ====================
    return launch.LaunchDescription([
        lead_screw_motor_node,          # 第一步：启动丝杠电机节点
        event_start_wheel_controller,    # 第二步：丝杠启动后启动底盘
        event_start_danger_command,      # 第三步：底盘启动后启动危险指令
        event_start_state_publisher      # 第四步：危险指令启动后启动状态发布
    ])