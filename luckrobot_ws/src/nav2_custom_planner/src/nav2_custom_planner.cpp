#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_custom_planner {

void CustomPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  tf_ = tf; node_ = parent.lock(); name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void CustomPlanner::cleanup() {}
void CustomPlanner::activate() {}
void CustomPlanner::deactivate() {}

double CustomPlanner::getHeuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  return std::hypot(static_cast<double>(x1) - x2, static_cast<double>(y1) - y2);
}

// 【动态避障核心修复：放宽碰撞判定阈值】
bool CustomPlanner::isNodeValid(unsigned int x, unsigned int y) {
  if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) return false;
  unsigned char cost = costmap_->getCost(x, y);
  
  // 从 100 放宽到 200。只要不撞到实体(254, 253)，小车就敢从旁边钻过去！
  return (cost < 200 && cost != nav2_costmap_2d::NO_INFORMATION);
}

std::vector<std::pair<int, int>> CustomPlanner::getHexNeighbors(unsigned int y) {
  if (y % 2 == 0) return {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}};
  else return {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {-1, 1}, {-1, -1}};
}

bool CustomPlanner::hasLineOfSight(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
  nav2_util::LineIterator line(x0, y0, x1, y1);
  for (; line.isValid(); line.advance()) {
    if (!isNodeValid(line.getX(), line.getY())) return false; 
  }
  return true; 
}

nav_msgs::msg::Path CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                              const geometry_msgs::msg::PoseStamped &goal) {
  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) 
      throw nav2_core::PlannerException("坐标系错误");

  unsigned int start_x, start_y, goal_x, goal_y;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) 
      throw nav2_core::PlannerException("目标不在地图内");

  std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareNode> open_list;
  std::vector<bool> closed_list(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(), false);
  open_list.push(std::make_shared<AStarNode>(start_x, start_y, 0.0, getHeuristic(start_x, start_y, goal_x, goal_y)));

  std::shared_ptr<AStarNode> current_node = nullptr;
  bool path_found = false;

  while (!open_list.empty()) {
    current_node = open_list.top(); open_list.pop();
    int index = current_node->y * costmap_->getSizeInCellsX() + current_node->x;
    if (closed_list[index]) continue;
    closed_list[index] = true;

    if (current_node->x == goal_x && current_node->y == goal_y) { path_found = true; break; }

    for (const auto& dir : getHexNeighbors(current_node->y)) {
      unsigned int next_x = current_node->x + dir.first;
      unsigned int next_y = current_node->y + dir.second;
      if (isNodeValid(next_x, next_y)) {
        int next_index = next_y * costmap_->getSizeInCellsX() + next_x;
        if (!closed_list[next_index]) {
          double penalty = static_cast<double>(costmap_->getCost(next_x, next_y)) / 254.0;
          
          // 【动态避障核心修复：降低惩罚权重】
          // 从 20.0 降到 5.0。减少系统对障碍物膨胀圈的恐惧，提升寻找出路的概率。
          double g_cost = current_node->g_cost + 1.0 + (penalty * 5.0); 
          
          double h_cost = getHeuristic(next_x, next_y, goal_x, goal_y);
          open_list.push(std::make_shared<AStarNode>(next_x, next_y, g_cost, h_cost, current_node));
        }
      }
    }
  }

  if (path_found && current_node != nullptr) {
    std::vector<geometry_msgs::msg::PoseStamped> reverse_path;
    while (current_node != nullptr) {
      geometry_msgs::msg::PoseStamped pose; double wx, wy;
      costmap_->mapToWorld(current_node->x, current_node->y, wx, wy);
      pose.pose.position.x = wx; pose.pose.position.y = wy; pose.pose.position.z = 0.0;
      pose.header.stamp = node_->now(); pose.header.frame_id = global_frame_;
      reverse_path.push_back(pose);
      current_node = current_node->parent;
    }
    std::reverse(reverse_path.begin(), reverse_path.end());

    // 视线平滑，消除细碎拐点
    if (reverse_path.size() > 2) {
      std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
      smoothed_path.push_back(reverse_path.front()); 

      size_t current_idx = 0;
      while (current_idx < reverse_path.size() - 1) {
        size_t next_idx = current_idx + 1;
        for (size_t target_idx = reverse_path.size() - 1; target_idx > current_idx; --target_idx) {
          unsigned int cx, cy, tx, ty;
          costmap_->worldToMap(reverse_path[current_idx].pose.position.x, reverse_path[current_idx].pose.position.y, cx, cy);
          costmap_->worldToMap(reverse_path[target_idx].pose.position.x, reverse_path[target_idx].pose.position.y, tx, ty);
          if (hasLineOfSight(cx, cy, tx, ty)) { next_idx = target_idx; break; }
        }
        smoothed_path.push_back(reverse_path[next_idx]);
        current_idx = next_idx;
      }
      
      std::vector<geometry_msgs::msg::PoseStamped> interpolated_path;
      for (size_t i = 0; i < smoothed_path.size() - 1; ++i) {
        auto p1 = smoothed_path[i]; auto p2 = smoothed_path[i+1];
        double dist = std::hypot(p2.pose.position.x - p1.pose.position.x, p2.pose.position.y - p1.pose.position.y);
        int num_points = std::max(1, static_cast<int>(std::ceil(dist / interpolation_resolution_)));
        for (int j = 0; j < num_points; ++j) {
          geometry_msgs::msg::PoseStamped pt = p1;
          pt.pose.position.x += j * (p2.pose.position.x - p1.pose.position.x) / num_points;
          pt.pose.position.y += j * (p2.pose.position.y - p1.pose.position.y) / num_points;
          interpolated_path.push_back(pt);
        }
      }
      interpolated_path.push_back(smoothed_path.back()); 
      global_path.poses = interpolated_path;
    } else {
      global_path.poses = reverse_path; 
    }

    global_path.poses.front() = start; 
    global_path.poses.back() = goal;   
    return global_path;

  } else {
    throw nav2_core::PlannerException("未能找到路径");
  }
}

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)