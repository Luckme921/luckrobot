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
  
  nav2_util::declare_parameter_if_not_declared(
      node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(0.7)); 
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);

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
  
  // 获取真正的终点
  auto final_goal = global_plan_.poses.back();
  double dist_to_final = std::hypot(final_goal.pose.position.x - pose_in_globalframe.pose.position.x, 
                                    final_goal.pose.position.y - pose_in_globalframe.pose.position.y);

  // 判定是否进入终点领域 (阈值必须小于 GoalChecker)
  bool at_xy_goal = (dist_to_final <= 0.12);

  double target_angle;
  if (at_xy_goal) {
      target_angle = tf2::getYaw(final_goal.pose.orientation);
  } else {
      target_angle = std::atan2(target_pose.pose.position.y - pose_in_globalframe.pose.position.y,
                                target_pose.pose.position.x - pose_in_globalframe.pose.position.x);
  }

  double angle_diff = calculateAngleDifference(pose_in_globalframe, target_angle);

  // 【保留原有的严苛控制逻辑】
  double start_turn_thresh = 0.15; 
  double stop_turn_thresh = 0.05;  

  if (at_xy_goal) {
      is_rotating_ = true; 
  } else {
      if (is_rotating_) {
        if (fabs(angle_diff) <= stop_turn_thresh) is_rotating_ = false;
      } else {
        if (fabs(angle_diff) >= start_turn_thresh) is_rotating_ = true;
      }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = node_->get_clock()->now();

  if (is_rotating_) {
    cmd_vel.twist.linear.x = 0.0; 
    
    // 【末点优化逻辑】：如果是在终点领域进行最后的对准
    if (at_xy_goal) {
        if (fabs(angle_diff) <= stop_turn_thresh) {
            // 对准后绝对刹死，等待 GoalChecker 判定
            cmd_vel.twist.angular.z = 0.0;
        } else {
            // 终点对准专用的激进参数：提高最低转速，缩短刹车距离
            double min_angular_speed_final = 0.20; // 较快的基础速度，拒绝磨叽
            double slowdown_range_final = 0.25;    // 离目标很近才开始减速
            double current_max_speed = std::min(max_angular_speed_, 0.70); 

            double current_abs_error = fabs(angle_diff);
            double target_angular_speed = current_max_speed;
            
            if (current_abs_error < slowdown_range_final) {
                double ratio = (current_abs_error - stop_turn_thresh) / (slowdown_range_final - stop_turn_thresh);
                ratio = std::max(0.0, std::min(1.0, ratio));
                target_angular_speed = min_angular_speed_final + ratio * (current_max_speed - min_angular_speed_final);
            }
            cmd_vel.twist.angular.z = (angle_diff > 0.0) ? -target_angular_speed : target_angular_speed;
        }
    } 
    // 【行进中的普通旋转】：保持原有缓慢减速逻辑，防止行进间车身晃动
    else {
        double min_angular_speed = 0.08; 
        double slowdown_range = 0.60;   
        double current_max_speed = std::min(max_angular_speed_, 0.70); 

        double current_abs_error = fabs(angle_diff);
        double target_angular_speed = current_max_speed;
        
        if (current_abs_error < slowdown_range) {
            double ratio = (current_abs_error - stop_turn_thresh) / (slowdown_range - stop_turn_thresh);
            ratio = std::max(0.0, std::min(1.0, ratio));
            target_angular_speed = min_angular_speed + ratio * (current_max_speed - min_angular_speed);
        }
        cmd_vel.twist.angular.z = (angle_diff > 0.0) ? -target_angular_speed : target_angular_speed;
    }
    
  } else {
    // 【保留原有的纯粹直行逻辑】
    cmd_vel.twist.linear.x = max_linear_speed_;
    cmd_vel.twist.angular.z = 0.0;
  }

  RCLCPP_INFO(node_->get_logger(), "状态:%s 发送速度(%.2f, %.2f) 角度偏差:%.2f",
              (at_xy_goal ? "终点对准" : (is_rotating_ ? "旋转" : "直行")), 
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
    const geometry_msgs::msg::PoseStamped &target_pose) { // <--- 关键恢复点
  float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  // 恢复回最稳妥的 atan2 计算，确保与你的旧版完全一致
  float target_angle = std::atan2(target_pose.pose.position.y - current_pose.pose.position.y,
                                  target_pose.pose.position.x - current_pose.pose.position.x);
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  else if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  return angle_diff;
}

// 重载一个只接收目标角度的版本，专供终点对准使用
double CustomController::calculateAngleDifference(
    const geometry_msgs::msg::PoseStamped &current_pose,
    double target_angle) {
  float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);
  double angle_diff = target_angle - current_robot_yaw;
  if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  else if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  return angle_diff;
}

} // namespace nav2_custom_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)