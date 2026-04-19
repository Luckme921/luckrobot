#!/usr/bin/env python3
import launch
import launch_ros.actions
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, TimerAction, LogInfo

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

    # ==================== 2. 底盘控制节点 ====================
    wheel_controller_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="wheel_controller_node",
        name="wheel_controller_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 3. 危险指令节点 ====================
    danger_command_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="danger_command_node",
        name="danger_command_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 4. 状态发布节点 ====================
    state_publisher_node = launch_ros.actions.Node(
        package=target_pkg,
        executable="state_publisher_node",
        name="state_publisher_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # ==================== 事件动作：严格物理延时顺序启动 ====================
    
    # 事件1：丝杠节点启动后 → 等待 3 秒 → 启动底盘控制节点
    event_start_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lead_screw_motor_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg=">>> 丝杠节点已分配PID 3秒，拉起底盘控制节点 <<<"),
                        wheel_controller_node
                    ]
                )
            ]
        )
    )

    # 事件2：底盘节点启动后 → 等待 2 秒 → 启动危险指令节点
    event_start_danger_command = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=wheel_controller_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        LogInfo(msg=">>> 底盘控制节点已分配PID 2秒，拉起危险报警节点 <<<"),
                        danger_command_node
                    ]
                )
            ]
        )
    )

    # 事件3：危险指令节点启动后 → 等待 2 秒 → 启动状态发布节点
    event_start_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=danger_command_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        LogInfo(msg=">>> 危险报警节点已分配PID 2秒，拉起状态发布节点 <<<"),
                        state_publisher_node
                    ]
                )
            ]
        )
    )

    # ==================== 组装启动描述 ====================
    return launch.LaunchDescription([
        LogInfo(msg=">>> 开始底层硬件节点顺序拉起序列 <<<"),
        lead_screw_motor_node,          # 第一步：直接启动丝杠节点
        event_start_wheel_controller,   # 注册监听器（后台待命）
        event_start_danger_command,     # 注册监听器（后台待命）
        event_start_state_publisher     # 注册监听器（后台待命）
    ])