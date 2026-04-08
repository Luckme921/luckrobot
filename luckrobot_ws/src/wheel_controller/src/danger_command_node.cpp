#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <mutex>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <thread>

// ==================== 配置常量 ====================
const char* SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
const int BAUDRATE = B115200;          
const int DATA_BITS = CS8;             
const int STOP_BITS = 0;               
const int PARITY = 0;                  

const uint8_t LEFT_DANGER_CMD[] = {0xAA, 0x55, 0x00, 0x0A, 0xFB};  
const uint8_t RIGHT_DANGER_CMD[] = {0xAA, 0x55, 0x00, 0x0B, 0xFB}; 
const int CMD_LENGTH = 5;              

const int COOLDOWN_MS = 3000;
const int SERIAL_WRITE_TIMEOUT_MS = 500; 

// [强化配置] 针对CH340的超长重试机制
const int MAX_INIT_RETRIES = 15;       // 最多尝试15次
const int RETRY_WAIT_MS = 2000;        // 每次等待2秒，给足USB初始化时间

// ==================== 核心节点类 ====================
class DangerCommandNode : public rclcpp::Node {
public:
    DangerCommandNode() : Node("danger_command_node"),
                          serial_fd_(-1),
                          left_dangerous_(false),
                          right_dangerous_(false),
                          last_left_send_ms_(0),
                          last_right_send_ms_(0) {
        // ==================== [强化修改开始] 带冷启动的串口初始化 ====================
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "正在尝试连接 CH340 设备...");
        RCLCPP_INFO(this->get_logger(), "目标: %s", SERIAL_PORT);
        RCLCPP_INFO(this->get_logger(), "========================================");

        bool connected = false;
        for (int i = 0; i < MAX_INIT_RETRIES && rclcpp::ok(); ++i) {
            RCLCPP_INFO(this->get_logger(), "[尝试 %d/%d] 正在打开串口...", i + 1, MAX_INIT_RETRIES);

            // 1. 确保上次的句柄已彻底关闭
            if (serial_fd_ >= 0) {
                close(serial_fd_);
                serial_fd_ = -1;
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 彻底释放
            }

            // 2. 尝试初始化
            if (init_serial()) {
                connected = true;
                RCLCPP_INFO(this->get_logger(), "========================================");
                RCLCPP_INFO(this->get_logger(), "串口连接成功！");
                RCLCPP_INFO(this->get_logger(), "========================================");
                break;
            } else {
                // 3. 失败处理
                if (i < MAX_INIT_RETRIES - 1) {
                    RCLCPP_WARN(this->get_logger(), "连接失败，%dms 后重试... (提示: 请检查USB线是否插紧)", RETRY_WAIT_MS);
                    std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_WAIT_MS));
                }
            }
        }

        if (!connected || serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "经过多次尝试，无法连接串口，节点退出。");
            rclcpp::shutdown();
            return;
        }
        // ==================== [强化修改结束] ====================

        // 订阅距离话题
        obstacle_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "obstacle_distances",
            10,
            std::bind(&DangerCommandNode::obstacle_callback, this, std::placeholders::_1));

        // 定时器：200ms检查
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DangerCommandNode::check_and_send, this));

        RCLCPP_INFO(this->get_logger(), "节点启动成功！冷却期: %dms", COOLDOWN_MS);
        RCLCPP_INFO(this->get_logger(), "串口: %s (115200 8N1)", SERIAL_PORT);
    }

    ~DangerCommandNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "节点退出");
    }

private:
    // 串口初始化（保持原样，仅被上面的逻辑调用）
    bool init_serial() {
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "  [错误] open() 失败: %s (errno: %d)", strerror(errno), errno);
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "  [错误] 获取属性失败: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // 串口核心配置
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= CS8 | CREAD | CLOCAL;

        cfsetispeed(&tty, BAUDRATE);
        cfsetospeed(&tty, BAUDRATE);

        // 超时配置
        tty.c_cc[VTIME] = SERIAL_WRITE_TIMEOUT_MS / 100;
        tty.c_cc[VMIN] = CMD_LENGTH;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "  [错误] 设置属性失败: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        tcflush(serial_fd_, TCIOFLUSH);
        return true;
    }

    // 指令转16进制
    std::string cmd_to_hex(const uint8_t* cmd, int len) {
        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0');
        for (int i = 0; i < len; ++i) {
            ss << std::setw(2) << static_cast<int>(cmd[i]) << " ";
        }
        return ss.str();
    }

    // 发送指令
    bool send_command(const uint8_t* cmd, int cmd_len) {
        if (serial_fd_ < 0 || !cmd || cmd_len <= 0) return false;

        tcflush(serial_fd_, TCOFLUSH);
        ssize_t sent = write(serial_fd_, cmd, cmd_len);
        
        if (sent != cmd_len) {
            RCLCPP_ERROR(this->get_logger(), "指令发送失败: 期望%d，实际%ld", cmd_len, sent);
            return false;
        }

        tcdrain(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "发送成功: %s", cmd_to_hex(cmd, cmd_len).c_str());
        return true;
    }

    // 距离回调
    void obstacle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (msg->data.size() >= 4) {
            left_dangerous_ = (msg->data[2] == 1.0);
            right_dangerous_ = (msg->data[3] == 1.0);
        } else {
            RCLCPP_WARN(this->get_logger(), "数据长度错误: 期望4，实际%ld", msg->data.size());
        }
    }

    // 核心冷却期逻辑
    void check_and_send() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        uint64_t now_ms = this->now().nanoseconds() / 1000000;

        // 左轮处理
        if (left_dangerous_) {
            uint64_t time_since_last = (last_left_send_ms_ == 0) ? COOLDOWN_MS + 1 : (now_ms - last_left_send_ms_);
            
            if (time_since_last >= COOLDOWN_MS) {
                RCLCPP_INFO(this->get_logger(), "左轮危险 | 距上次发送%ldms（冷却期%dms）→ 发送指令", 
                             time_since_last, COOLDOWN_MS);
                if (send_command(LEFT_DANGER_CMD, CMD_LENGTH)) {
                    last_left_send_ms_ = now_ms;
                }
            }
        }

        // 右轮处理
        if (right_dangerous_) {
            uint64_t time_since_last = (last_right_send_ms_ == 0) ? COOLDOWN_MS + 1 : (now_ms - last_right_send_ms_);
            
            if (time_since_last >= COOLDOWN_MS) {
                RCLCPP_INFO(this->get_logger(), "右轮危险 | 距上次发送%ldms（冷却期%dms）→ 发送指令", 
                             time_since_last, COOLDOWN_MS);
                if (send_command(RIGHT_DANGER_CMD, CMD_LENGTH)) {
                    last_right_send_ms_ = now_ms;
                }
            }
        }
    }

    // 成员变量
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    
    int serial_fd_;                         
    bool left_dangerous_;                   
    bool right_dangerous_;                  
    uint64_t last_left_send_ms_;            
    uint64_t last_right_send_ms_;           
    std::mutex state_mutex_;                
};

// ==================== 主函数 ====================
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DangerCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}