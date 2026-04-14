#ifndef NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_custom_controller {

class CustomController : public nav2_core::Controller {
public:
  CustomController() = default;
  ~CustomController() override = default;
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

protected:
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D *costmap_;
  nav_msgs::msg::Path global_plan_;
  
  // 核心控制参数
  double max_angular_speed_;
  double max_linear_speed_;
  double lookahead_dist_; // 新增：前视距离

  // 状态机标志位：当前是否处于“原地旋转”状态
  bool is_rotating_ = false;

  // 获取路径中前方一定距离的前视目标点
  geometry_msgs::msg::PoseStamped
  getNearestTargetPose(const geometry_msgs::msg::PoseStamped &current_pose);
  
  // 计算目标点方向和当前位置的角度差
  double
  calculateAngleDifference(const geometry_msgs::msg::PoseStamped &current_pose,
                           const geometry_msgs::msg::PoseStamped &target_pose);
};

} // namespace nav2_custom_controller

#endif // NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_