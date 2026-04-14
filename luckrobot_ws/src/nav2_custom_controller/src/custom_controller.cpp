#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <cmath>

namespace nav2_custom_controller {
void CustomController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;

  // 严格按照你的硬件限制，默认锁死线速度 0.1，角速度 1.0
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

  // 设置前视距离：看向前方 0.4 米处，防止近视眼导致的疯狂超调
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
}

void CustomController::cleanup() {
  RCLCPP_INFO(node_->get_logger(), "清理控制器：%s", plugin_name_.c_str());
}

void CustomController::activate() {
  RCLCPP_INFO(node_->get_logger(), "激活控制器：%s", plugin_name_.c_str());
  is_rotating_ = false; // 激活时重置状态
}

void CustomController::deactivate() {
  RCLCPP_INFO(node_->get_logger(), "停用控制器：%s", plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
  
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("收到长度为零的路径");
  }

  // 1. 将机器人当前姿态转换到全局计划坐标系中
  geometry_msgs::msg::PoseStamped pose_in_globalframe;
  if (!nav2_util::transformPoseInTargetFrame(
          pose, pose_in_globalframe, *tf_, global_plan_.header.frame_id, 0.1)) {
    throw nav2_core::PlannerException("无法将机器人姿态转换为全局坐标系");
  }

  // 2. 获取前视目标点
  auto target_pose = getNearestTargetPose(pose_in_globalframe);
  
  // 3. 计算角度差
  auto angle_diff = calculateAngleDifference(pose_in_globalframe, target_pose);

  // 4. 【核心迟滞状态机】严格控制输出
  double start_turn_thresh = 0.20; // 约 11.5 度，偏差大于此值开始原地转
  double stop_turn_thresh = 0.05;  // 约 2.8 度，转到偏差小于此值才允许直行

  if (is_rotating_) {
    // 如果正在转，只有对得很准了才能停下
    if (fabs(angle_diff) < stop_turn_thresh) {
      is_rotating_ = false;
    }
  } else {
    // 如果正在直行，只有偏得比较多了才允许停下开始转
    if (fabs(angle_diff) > start_turn_thresh) {
      is_rotating_ = true;
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id; // 必须是 base_link
  cmd_vel.header.stamp = node_->get_clock()->now();

  // 严格执行你的硬件约束
  if (is_rotating_) {
    cmd_vel.twist.linear.x = 0.0;
    // 根据偏差方向，输出正 1.0 或 负 1.0
    cmd_vel.twist.angular.z = (angle_diff > 0.0) ? max_angular_speed_ : -max_angular_speed_;
  } else {
    // 纯直行
    cmd_vel.twist.linear.x = max_linear_speed_;
    cmd_vel.twist.angular.z = 0.0;
  }

  RCLCPP_INFO(node_->get_logger(), "发送速度(%.2f, %.2f) 角度偏差:%.2f",
              cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, angle_diff);
              
  return cmd_vel;
}

void CustomController::setSpeedLimit(const double &, const bool &) {}

void CustomController::setPlan(const nav_msgs::msg::Path &path) {
  global_plan_ = path;
}

geometry_msgs::msg::PoseStamped CustomController::getNearestTargetPose(
    const geometry_msgs::msg::PoseStamped &current_pose) {
  using nav2_util::geometry_utils::euclidean_distance;
  
  // 1. 找到距离当前车体最近的点
  int nearest_pose_index = 0;
  double min_dist = euclidean_distance(current_pose, global_plan_.poses.front());
  for (size_t i = 1; i < global_plan_.poses.size(); i++) {
    double dist = euclidean_distance(current_pose, global_plan_.poses[i]);
    if (dist < min_dist) {
      nearest_pose_index = i;
      min_dist = dist;
    }
  }

  // 2. 擦除已经走过的路径
  global_plan_.poses.erase(global_plan_.poses.begin(),
                           global_plan_.poses.begin() + nearest_pose_index);

  // 3. 【修复原逻辑缺陷】：往后寻找距离车体 lookahead_dist_ 的点
  for (size_t i = 0; i < global_plan_.poses.size(); i++) {
      double dist = euclidean_distance(current_pose, global_plan_.poses[i]);
      if (dist >= lookahead_dist_) {
          return global_plan_.poses[i];
      }
  }
  
  // 如果路径剩下的长度不足前视距离，直接返回最后一个点（终点）
  return global_plan_.poses.back();
}

double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &target_pose) {
  
  float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  float target_angle =
      std::atan2(target_pose.pose.position.y - current_pose.pose.position.y,
                 target_pose.pose.position.x - current_pose.pose.position.x);
                 
  double angle_diff = target_angle - current_robot_yaw;
  
  // 角度归一化到 -PI 到 PI
  if (angle_diff < -M_PI) {
    angle_diff += 2.0 * M_PI;
  } else if (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  }
  return angle_diff;
}
} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)