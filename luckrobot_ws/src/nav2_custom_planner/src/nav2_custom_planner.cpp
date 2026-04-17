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

// 找路时的底线判定
bool CustomPlanner::isNodeValid(unsigned int x, unsigned int y) {
  if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) return false;
  unsigned char cost = costmap_->getCost(x, y);
  return (cost < 253 && cost != nav2_costmap_2d::NO_INFORMATION);
}

// 视线平滑时的安全底线
bool CustomPlanner::isSightValid(unsigned int x, unsigned int y) {
  if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) return false;
  unsigned char cost = costmap_->getCost(x, y);
  return (cost < 180 && cost != nav2_costmap_2d::NO_INFORMATION);
}

std::vector<std::pair<int, int>> CustomPlanner::getHexNeighbors(unsigned int y) {
  if (y % 2 == 0) return {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}};
  else return {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {-1, 1}, {-1, -1}};
}

bool CustomPlanner::hasLineOfSight(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) {
  nav2_util::LineIterator line(x0, y0, x1, y1);
  for (; line.isValid(); line.advance()) {
    if (!isSightValid(line.getX(), line.getY())) return false; 
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
          
          // 【核心改造1：抛弃最短路径，开启指数级安全惩罚】
          // 哪怕稍微偏离正中间一点点，代价也会爆炸增长，逼迫算法在最空旷的地带画线
          double normalized_cost = static_cast<double>(costmap_->getCost(next_x, next_y)) / 254.0;
          double safety_penalty = normalized_cost * normalized_cost * 100.0; 
          
          // 【核心改造2：转向惩罚】
          // 如果搜索方向发生了改变，加上严厉的惩罚，逼迫算法生成长直线，减少小车自转需求
          double turn_penalty = 0.0;
          if (current_node->parent != nullptr) {
            int old_dx = static_cast<int>(current_node->x) - static_cast<int>(current_node->parent->x);
            int old_dy = static_cast<int>(current_node->y) - static_cast<int>(current_node->parent->y);
            int new_dx = static_cast<int>(next_x) - static_cast<int>(current_node->x);
            int new_dy = static_cast<int>(next_y) - static_cast<int>(current_node->y);
            if (old_dx != new_dx || old_dy != new_dy) {
                turn_penalty = 3.0; // 一旦转弯就罚分
            }
          }

          // 总代价值：基础距离 + 极度怕死的安全惩罚 + 极度讨厌转弯的惩罚
          double g_cost = current_node->g_cost + 1.0 + safety_penalty + turn_penalty; 
          
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