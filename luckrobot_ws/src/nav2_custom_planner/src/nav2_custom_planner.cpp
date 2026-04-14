#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner {

void CustomPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  tf_ = tf;
  node_ = parent.lock();
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void CustomPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "清理插件 %s", name_.c_str()); }
void CustomPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "激活插件 %s", name_.c_str()); }
void CustomPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "停用插件 %s", name_.c_str()); }

double CustomPlanner::getHeuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  return std::hypot(static_cast<double>(x1) - x2, static_cast<double>(y1) - y2);
}

bool CustomPlanner::isNodeValid(unsigned int x, unsigned int y) {
  if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) return false;
  unsigned char cost = costmap_->getCost(x, y);
  // 严格避障：拒绝致命障碍物和内切膨胀区
  return (cost != nav2_costmap_2d::LETHAL_OBSTACLE && 
          cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE && 
          cost != nav2_costmap_2d::NO_INFORMATION);
}

// 【核心改造】蜂窝网格 (Hexagonal Grid) 的邻居映射
std::vector<std::pair<int, int>> CustomPlanner::getHexNeighbors(unsigned int y) {
  if (y % 2 == 0) {
    // 偶数行 (y=0, 2, 4...) 的 6 个邻居
    return {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}};
  } else {
    // 奇数行 (y=1, 3, 5...) 的 6 个邻居
    return {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {-1, 1}, {-1, -1}};
  }
}

nav_msgs::msg::Path CustomPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) {

  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("坐标系必须是全局坐标系");
  }

  unsigned int start_x, start_y, goal_x, goal_y;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    throw nav2_core::PlannerException("起点或终点不在地图内");
  }

  std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareNode> open_list;
  std::vector<bool> closed_list(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(), false);

  open_list.push(std::make_shared<AStarNode>(start_x, start_y, 0.0, getHeuristic(start_x, start_y, goal_x, goal_y)));

  std::shared_ptr<AStarNode> current_node = nullptr;
  bool path_found = false;

  while (!open_list.empty()) {
    current_node = open_list.top();
    open_list.pop();

    int index = current_node->y * costmap_->getSizeInCellsX() + current_node->x;
    if (closed_list[index]) continue;
    closed_list[index] = true;

    if (current_node->x == goal_x && current_node->y == goal_y) {
      path_found = true;
      break;
    }

    // 使用六边形规则获取邻居
    for (const auto& dir : getHexNeighbors(current_node->y)) {
      unsigned int next_x = current_node->x + dir.first;
      unsigned int next_y = current_node->y + dir.second;

      if (isNodeValid(next_x, next_y)) {
        int next_index = next_y * costmap_->getSizeInCellsX() + next_x;
        if (!closed_list[next_index]) {
          
          // 六边形网格中，移动到相邻 6 个格子的物理距离成本视为完全相等 (1.0)
          // 这是蜂窝网格之所以“圆滑”的物理学基础！
          double move_cost = 1.0;
          
          // 读取代价地图的值 (0~254)
          double penalty = static_cast<double>(costmap_->getCost(next_x, next_y)) / 254.0;
          
          // 【动态避障核心】：给障碍物极高的惩罚权重 (* 15.0)
          // 哪怕路人只是被雷达扫到一点点外边，A* 也会像躲避瘟疫一样绕开他
          double g_cost = current_node->g_cost + move_cost + (penalty * 15.0); 
          double h_cost = getHeuristic(next_x, next_y, goal_x, goal_y);
          
          open_list.push(std::make_shared<AStarNode>(next_x, next_y, g_cost, h_cost, current_node));
        }
      }
    }
  }

  if (path_found && current_node != nullptr) {
    std::vector<geometry_msgs::msg::PoseStamped> reverse_path;
    while (current_node != nullptr) {
      geometry_msgs::msg::PoseStamped pose;
      double wx, wy;
      costmap_->mapToWorld(current_node->x, current_node->y, wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      reverse_path.push_back(pose);
      current_node = current_node->parent;
    }
    std::reverse(reverse_path.begin(), reverse_path.end());
    global_path.poses = reverse_path;
    global_path.poses.front() = start;
    global_path.poses.back() = goal;
    return global_path;
  } else {
    throw nav2_core::PlannerException("蜂窝 A* 未能找到路径");
  }
}

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)