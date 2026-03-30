#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <atomic>
#include <cmath>

// 严格匹配URDF & 你的配置
#define LEFT_WHEEL_JOINT    "whell_left_joint"
#define RIGHT_WHEEL_JOINT   "wheel_right_joint"
#define SCREW_JOINT         "screw_joint"

// 核心要求：移动速度 1cm/s = 10mm/s
const float MOVE_SPEED_MM_PER_SEC = 10.0f;
// 控制频率 50Hz (20ms)
const float CONTROL_PERIOD = 0.02f;

class RobotStatePublisher : public rclcpp::Node
{
public:
    RobotStatePublisher() : Node("state_publisher_node")
    {
        // 初始化：丝杠在顶端(0mm)，当前位置=目标位置=0
        wheel_left_ = 0.0;
        wheel_right_ = 0.0;
        current_pos_mm_ = 0.0f;   // 当前位置(mm)
        target_pos_mm_ = 0.0f;    // 目标位置(mm)

        // 轮子订阅（无闪烁）
        wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_position", 100,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if(msg->data.size() >= 2){
                    wheel_left_ = -msg->data[0] * 0.0174533;
                    wheel_right_ = -msg->data[1] * 0.0174533;
                }
            }
        );

        // 核心：增量式丝杠控制（方向修正+累加 + 无软限位）
        screw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/displacement", 100,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                // 方向反转：负值=向下
                float delta = -msg->data;
                // 修复原子变量累加操作
                target_pos_mm_.store(target_pos_mm_.load() + delta);
                
                RCLCPP_INFO(this->get_logger(), 
                    "增量: %.0fmm | 目标位置: %.0fmm", 
                    delta, target_pos_mm_.load());
            }
        );

        // 发布关节状态
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);

        // 50Hz平滑控制定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotStatePublisher::smooth_control, this)
        );

        RCLCPP_INFO(this->get_logger(), "✅ 节点启动成功");
        RCLCPP_INFO(this->get_logger(), "✅ 方向：负值=向下");
        RCLCPP_INFO(this->get_logger(), "✅ 速度：1cm/s");
        RCLCPP_INFO(this->get_logger(), "✅ 控制：增量累加模式");
        RCLCPP_INFO(this->get_logger(), "✅ 已移除：软限位保护(无行程限制)");
    }

private:
    // 匀速平滑运动控制
    void smooth_control()
    {
        // 每帧移动步长
        float step = MOVE_SPEED_MM_PER_SEC * CONTROL_PERIOD;
        float curr = current_pos_mm_.load();
        float target = target_pos_mm_.load();

        // 平滑趋近目标位置（无任何限位）
        if (curr < target) {
            current_pos_mm_.store(curr + step);
        } else if (curr > target) {
            current_pos_mm_.store(curr - step);
        }

        // 发布关节数据（mm转m）
        publish_joint_state();
    }

    // 发布joint_states
    void publish_joint_state()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();

        msg.name = {LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT, SCREW_JOINT};
        msg.position = {
            wheel_left_.load(std::memory_order_relaxed),
            wheel_right_.load(std::memory_order_relaxed),
            current_pos_mm_.load() / 1000.0f  // 毫米转米
        };

        joint_pub_->publish(msg);
    }

    // 通信句柄
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr screw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 线程安全变量
    std::atomic<double> wheel_left_;
    std::atomic<double> wheel_right_;
    std::atomic<float> current_pos_mm_;  // 当前位置(mm)
    std::atomic<float> target_pos_mm_;   // 目标位置(mm)
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatePublisher>());
    rclcpp::shutdown();
    return 0;
}