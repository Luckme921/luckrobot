#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp> // 🔥 修正：ROS 2 中 UInt8 的头文件叫 u_int8.hpp
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <map>
#include <string>
#include <cmath>
#include <algorithm>
#include <cctype>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 定义四自由度路点：底盘(X,Y,Yaw) + 丝杠高度(Z)
struct Waypoint {
    double x;
    double y;
    double yaw;         
    float screw_height; 
};

class NavManagerNode : public rclcpp::Node {
public:
    NavManagerNode() : Node("nav_manager_node") {
        
        target_locations_["原点"]   = { 0.0,  0.0,  0.0, -462.0};
        target_locations_["餐桌"]   = { 0.875, 1.436, -2.546, 0.0};
        target_locations_["桌子"]   = { 3.116, 2.368, -1.550, -200.0};
        target_locations_["电视"]   = { 3.349, 3.031,  1.593, -250.0};
        target_locations_["冰箱"]       = { 2.105, 4.050,  2.180, -100.0};
        target_locations_["水池"]   = { 3.294, 4.550,  1.167,  0.0};
        target_locations_["沙发"] = { 2.257, 1.117, -1.100, -300.0};
        target_locations_["包装区"]     = { 3.2, -1.5,  0.00, -350.0};
        target_locations_["工位A"]      = { 6.1,  0.5,  1.57, -150.0};
        target_locations_["工位B"]      = { 6.1,  2.5,  1.57, -150.0};
        target_locations_["工位C"]      = { 6.1,  4.5,  1.57, -150.0};
        target_locations_["工具架"]     = {-2.5,  1.0, -1.57, -450.0};
        target_locations_["会议室"]     = { 8.0,  8.0,  0.00, -220.0};
        target_locations_["前台"]       = { 0.0,  8.0,  3.14, -100.0};
        target_locations_["饮水机"]     = {-1.5, -2.5,  1.57, -380.0};
        target_locations_["书架顶层"]   = { 2.0,  2.0,  0.00, -460.0};
        target_locations_["书架底层"]   = { 2.0,  2.0,  0.00,  -50.0};
        target_locations_["沙发"]       = {-3.0,  2.0,  1.57, -120.0};
        target_locations_["阳台"]       = { 4.0,  7.0, -1.57, -200.0};
        target_locations_["垃圾桶"]     = { 0.5, -1.0,  3.14, -400.0};

        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        screw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lead_screw/displacement", 10);
        vla_trigger_pub_ = this->create_publisher<std_msgs::msg::String>("/vla_task_trigger", 10);
        
        // 控制语音硬件的话题发布器
        voice_prompt_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/voice_prompt", 10);

        voice_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/user_voice_cmd", 10,
            std::bind(&NavManagerNode::voice_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "🚀 中央调度节点启动成功！(已集成任务语音播报)");
    }

private:
    std::map<std::string, Waypoint> target_locations_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr screw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vla_trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr voice_prompt_pub_; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_sub_;
    
    std::string current_task_name_;
    float current_screw_height_target_; 

    void yaw_to_quaternion(double yaw, double& qx, double& qy, double& qz, double& qw) {
        qx = 0.0; qy = 0.0;
        qz = std::sin(yaw / 2.0);
        qw = std::cos(yaw / 2.0);
    }

    void voice_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        
        std::string lower_command = command;
        std::transform(lower_command.begin(), lower_command.end(), lower_command.begin(),
            [](unsigned char c){ return std::tolower(c); });

        if (lower_command.find("lucky") == std::string::npos) {
            RCLCPP_DEBUG(this->get_logger(), "🛑 未检测到唤醒词 'Lucky'，忽略日常对话: [%s]", command.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✨ 机器人被唤醒！收到有效指令: [%s]", command.c_str());

        bool location_found = false;
        std::string target_name = "";
        Waypoint target_wp;

        for (const auto& location : target_locations_) {
            if (command.find(location.first) != std::string::npos) {
                location_found = true;
                target_name = location.first;
                target_wp = location.second;
                current_task_name_ = command; 
                current_screw_height_target_ = target_wp.screw_height; 
                break; 
            }
        }

        if (!location_found) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 听到了唤醒词，但未在指令中发现已知的预设地点！小车保持原地不动。");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🎯 成功解析目标地点: [%s]", target_name.c_str());
        RCLCPP_INFO(this->get_logger(), "   - Nav2 坐标 : X=%.2f, Y=%.2f, Yaw=%.2f", target_wp.x, target_wp.y, target_wp.yaw);
        RCLCPP_INFO(this->get_logger(), "   - 待执行丝杠高度 : %.1f mm (将在导航抵达后执行)", target_wp.screw_height);

        // 调用 Nav2 执行底盘移动
        send_nav_goal(target_wp.x, target_wp.y, target_wp.yaw);
    }

    void send_nav_goal(double x, double y, double yaw) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "❌ Nav2 Action 服务器未就绪！无法导航！");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        double qx, qy, qz, qw;
        yaw_to_quaternion(yaw, qx, qy, qz, qw);
        goal_msg.pose.pose.orientation.x = qx;
        goal_msg.pose.pose.orientation.y = qy;
        goal_msg.pose.pose.orientation.z = qz;
        goal_msg.pose.pose.orientation.w = qw;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                    {
                        RCLCPP_INFO(this->get_logger(), "✅ 导航圆满到达目的地！底盘已停稳。");
                        
                        // 动作2：到达目的地，下发语音 [0x0E] -> "到达目标点成功vla开始"
                        std_msgs::msg::UInt8 voice_arrived_msg;
                        voice_arrived_msg.data = 0x0E;
                        voice_prompt_pub_->publish(voice_arrived_msg);
                        
                        RCLCPP_INFO(this->get_logger(), "⚙️ 开始执行丝杠升降动作，目标高度: %.1f mm", current_screw_height_target_);
                        {
                            std_msgs::msg::Float32 screw_msg;
                            screw_msg.data = current_screw_height_target_;
                            screw_pub_->publish(screw_msg);
                        }

                        RCLCPP_INFO(this->get_logger(), "📡 正在向 10.0.0.51 (VLA节点) 下发视觉作业指令...");
                        
                        {
                            std_msgs::msg::String vla_msg;
                            vla_msg.data = current_task_name_;
                            vla_trigger_pub_->publish(vla_msg);
                        }
                        break;
                    }
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "❌ 导航被强行终止！(可能由于障碍物死锁)");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "⚠️ 导航任务被外部取消。");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "❌ 导航失败，未知原因！");
                        break;
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "🚙 路线已下发至 Nav2，机器人开始安全移动...");
        
        // 动作1：开始导航，下发语音 [0x0D] -> "唤醒成功，导航开始"
        std_msgs::msg::UInt8 voice_start_msg;
        voice_start_msg.data = 0x0D;
        voice_prompt_pub_->publish(voice_start_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavManagerNode>());
    rclcpp::shutdown();
    return 0;
}