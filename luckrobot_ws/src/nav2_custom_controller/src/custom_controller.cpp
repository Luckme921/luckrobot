#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
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

  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  
  // 限制最高转速为 0.7，防止巨大惯性导致甩过头
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(0.7)); 
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

  // 【治标核心】前视距离拉大到 0.8 米，无视微小偏离
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
}

void CustomController::cleanup() { RCLCPP_INFO(node_->get_logger(), "清理控制器"); }
void CustomController::activate() { is_rotating_ = false; }
void CustomController::deactivate() {}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
  
  if (global_plan_.poses.empty()) throw nav2_core::PlannerException("路径为空");

  geometry_msgs::msg::PoseStamped pose_in_globalframe;
  if (!nav2_util::transformPoseInTargetFrame(pose, pose_in_globalframe, *tf_, global_plan_.header.frame_id, 0.1)) {
    throw nav2_core::PlannerException("TF转换失败");
  }

  auto target_pose = getNearestTargetPose(pose_in_globalframe);
  auto angle_diff = calculateAngleDifference(pose_in_globalframe, target_pose);

  // 【终极迟滞区间】：偏离超过 28度(0.5rad) 才转，对准到 3度(0.05rad) 以内才直行
  double start_turn_thresh = 0.50; 
  double stop_turn_thresh = 0.05;  

  if (is_rotating_) {
    if (fabs(angle_diff) <= stop_turn_thresh) is_rotating_ = false;
  } else {
    if (fabs(angle_diff) >= start_turn_thresh) is_rotating_ = true;
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_->get_clock()->now();

  if (is_rotating_) {
    cmd_vel.twist.linear.x = 0.0;
    
    // 【比例减速逻辑】最后阶段柔和刹车，稳稳停住
    double min_angular_speed = 0.10; // 极低速保底
    double slowdown_range = 0.60;    // 距离目标 34度 就开始慢慢踩刹车
    double current_max_speed = std::min(max_angular_speed_, 0.70); 

    double current_abs_error = fabs(angle_diff);
    double target_angular_speed = current_max_speed;
    
    if (current_abs_error < slowdown_range) {
        double ratio = (current_abs_error - stop_turn_thresh) / (slowdown_range - stop_turn_thresh);
        ratio = std::max(0.0, std::min(1.0, ratio));
        target_angular_speed = min_angular_speed + ratio * (current_max_speed - min_angular_speed);
    }
    
    // 极性映射：目标在左(正差)，发负速度左转；目标在右(负差)，发正速度右转
    cmd_vel.twist.angular.z = (angle_diff > 0.0) ? -target_angular_speed : target_angular_speed;
    
  } else {
    cmd_vel.twist.linear.x = max_linear_speed_;
    cmd_vel.twist.angular.z = 0.0;
  }

  RCLCPP_INFO(node_->get_logger(), "状态:%s 发送速度(%.2f, %.2f) 角度偏差:%.2f",
              is_rotating_ ? "旋转" : "直行", 
              cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, angle_diff);
              
  return cmd_vel;
}

void CustomController::setSpeedLimit(const double &, const bool &) {}
void CustomController::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

geometry_msgs::msg::PoseStamped CustomController::getNearestTargetPose(
    const geometry_msgs::msg::PoseStamped &current_pose) {
  
  if (global_plan_.poses.size() < 2) return global_plan_.poses.back();

  int nearest_idx = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < global_plan_.poses.size(); i++) {
    double dist = std::hypot(global_plan_.poses[i].pose.position.x - current_pose.pose.position.x, 
                             global_plan_.poses[i].pose.position.y - current_pose.pose.position.y);
    if (dist < min_dist) { min_dist = dist; nearest_idx = i; }
  }

  double accum_dist = 0.0;
  int target_idx = nearest_idx;
  for (size_t i = nearest_idx; i < global_plan_.poses.size() - 1; i++) {
    accum_dist += std::hypot(global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x, 
                             global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y);
    if (accum_dist >= lookahead_dist_) { target_idx = i + 1; break; }
  }

  if (target_idx == nearest_idx && !global_plan_.poses.empty()) target_idx = global_plan_.poses.size() - 1;

  if (nearest_idx > 0) {
    global_plan_.poses.erase(global_plan_.poses.begin(), global_plan_.poses.begin() + nearest_idx);
    target_idx -= nearest_idx;
  }
  return global_plan_.poses[target_idx];
}

double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &target_pose) {
  float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  float target_angle = std::atan2(target_pose.pose.position.y - current_pose.pose.position.y,
                                  target_pose.pose.position.x - current_pose.pose.position.x);
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  else if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  return angle_diff;
}
} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)