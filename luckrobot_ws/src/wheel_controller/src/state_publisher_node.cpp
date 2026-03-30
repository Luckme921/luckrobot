#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <atomic>
#include <cmath>
#include <queue>
#include <mutex>
#define PI 3.14159265358979323846

// 固定配置
#define LEFT_JOINT   "whell_left_joint"
#define RIGHT_JOINT  "wheel_right_joint"
#define SCREW_JOINT  "screw_joint"

// 丝杠参数
const float INIT_SPEED = 20.0f;
const float NORMAL_SPEED = 50.0f;
const float DT = 0.02f;
const float START_POS = 470.0f;
const float ZERO_POS = 0.0f;

const double THRESH = 0.1;

enum State { INIT, RUN };

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher() : Node("joint_pub")
    {
        wheel_l_ = 0.0;
        wheel_r_ = 0.0;
        screw_pos_ = START_POS;
        target_ = ZERO_POS;
        state_ = INIT;

        // 轮子订阅：仅过滤0.0脏数据
        wheel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_position", 100,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 2) return;
                double l = msg->data[0];
                double r = msg->data[1];

                if (l == 0.0 && std::fabs(wheel_l_.load()) > THRESH) {}
                else wheel_l_ = l;

                if (r == 0.0 && std::fabs(wheel_r_.load()) > THRESH) {}
                else wheel_r_ = r;
            }
        );

        // 丝杠指令
        screw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/displacement", 100,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mtx_);
                q_.push(-msg->data);
            }
        );

        // 50Hz发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&JointPublisher::pub, this)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }

private:
    void pub()
    {
        // 丝杠运动
        float curr = screw_pos_.load();
        float tgt = target_.load();

        if (state_ == INIT) {
            if (curr > tgt) screw_pos_ = curr - INIT_SPEED * DT;
            else { screw_pos_ = ZERO_POS; state_ = RUN; }
        } else {
            if (std::fabs(curr - tgt) < 0.1f) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (!q_.empty()) { target_ = tgt + q_.front(); q_.pop(); }
            } else {
                screw_pos_ = curr + (curr < tgt ? 1 : -1) * NORMAL_SPEED * DT;
            }
        }

        // 角度 → 弧度转换（唯一修改的地方）
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = get_clock()->now();
        msg.name = {LEFT_JOINT, RIGHT_JOINT, SCREW_JOINT};
        msg.position = {
            wheel_l_.load() * PI / 180.0,   // 角度转弧度
            wheel_r_.load() * PI / 180.0,   // 角度转弧度
            screw_pos_.load() / 1000.0f
        };
        pub_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr screw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<double> wheel_l_, wheel_r_;
    std::atomic<float> screw_pos_, target_;
    std::atomic<State> state_;
    std::queue<float> q_;
    std::mutex mtx_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}