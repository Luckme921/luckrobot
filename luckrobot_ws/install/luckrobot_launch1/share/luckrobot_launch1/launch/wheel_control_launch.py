#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler

def generate_launch_description():
    target_pkg = "wheel_controller"

    # ==================== 1. 底盘控制节点（最先启动） ====================
    wheel_controller_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="wheel_controller_node",
        name="wheel_controller_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 2. 丝杠电机节点（底盘节点启动后执行） ====================
    lead_screw_motor_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="lead_screw_motor_node",
        name="lead_screw_motor_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 3. 危险指令节点（丝杠节点启动后执行） ====================
    danger_command_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="danger_command_node",
        name="danger_command_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 事件动作：严格顺序启动 ====================
    # 事件1：底盘节点启动后 → 启动丝杠电机节点
    event_start_lead_screw = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=wheel_controller_node,  # 监听底盘节点启动
            on_start=[lead_screw_motor_node]       # 启动后执行的动作
        )
    )

    # 事件2：丝杠节点启动后 → 启动危险指令节点
    event_start_danger_command = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lead_screw_motor_node,  # 监听丝杠节点启动
            on_start=[danger_command_node]        # 启动后执行的动作
        )
    )

    # ==================== 组装启动描述 ====================
    return launch.LaunchDescription([
        wheel_controller_node,          # 第一步：启动底盘节点
        event_start_lead_screw,         # 第二步：底盘启动后启动丝杠
        event_start_danger_command      # 第三步：丝杠启动后启动危险指令
    ])