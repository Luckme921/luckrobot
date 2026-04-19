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

const int MAX_INIT_RETRIES = 15;       
const int RETRY_WAIT_MS = 2000;        

// ==================== 核心节点类 ====================
class DangerCommandNode : public rclcpp::Node {
public:
    DangerCommandNode() : Node("danger_command_node"),
                          serial_fd_(-1),
                          left_dangerous_(false),
                          right_dangerous_(false),
                          last_left_send_ms_(0),
                          last_right_send_ms_(0),
                          retry_count_(0) {
        
        RCLCPP_INFO(this->get_logger(), "🚀 危险报警节点启动 (异步自动重连版)");

        // 订阅距离话题
        obstacle_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "obstacle_distances",
            10,
            std::bind(&DangerCommandNode::obstacle_callback, this, std::placeholders::_1));

        // 业务检查定时器：200ms检查一次危险状态
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DangerCommandNode::check_and_send, this));

        // 🔥 [核心优化] 异步初始化定时器：不再阻塞构造函数，后台每 2 秒尝试一次连接
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(RETRY_WAIT_MS),
            std::bind(&DangerCommandNode::async_init_serial, this));

        // 启动时立刻触发一次连接尝试
        async_init_serial();
    }

    ~DangerCommandNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "节点安全退出");
    }

private:
    // 🔥 [核心优化] 后台异步重连逻辑
    void async_init_serial() {
        // 如果已经连上，直接返回
        if (serial_fd_ >= 0) return;

        if (retry_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "正在尝试连接 CH340 串口设备...");
        }

        if (init_serial()) {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "✅ 串口连接成功！(冷却期: %dms)", COOLDOWN_MS);
            RCLCPP_INFO(this->get_logger(), "========================================");
            
            // 连上后，停止重连定时器，清空重试次数
            if (init_timer_) {
                init_timer_->cancel();
            }
            retry_count_ = 0;
        } else {
            retry_count_++;
            if (retry_count_ >= MAX_INIT_RETRIES) {
                RCLCPP_FATAL(this->get_logger(), "❌ 经过 %d 次尝试仍无法连接，彻底放弃，节点终止！", MAX_INIT_RETRIES);
                rclcpp::shutdown();
            } else {
                RCLCPP_WARN(this->get_logger(), "⏳ 连接失败，将在后台重试 (%d/%d)...", retry_count_, MAX_INIT_RETRIES);
            }
        }
    }

    // 串口底层初始化
    bool init_serial() {
        // 🔥 [核心优化] 增加 O_NDELAY 标志。
        // CH340 经常会因为没有载波信号导致 open() 函数死锁挂起。O_NDELAY 强制立刻打开！
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            return false; // 打开失败，交由外层重连
        }

        // 打开成功后，恢复阻塞模式，供后续读写使用
        fcntl(serial_fd_, F_SETFL, 0);

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
        // 如果没连上，不执行发送
        if (serial_fd_ < 0 || !cmd || cmd_len <= 0) return false;

        tcflush(serial_fd_, TCOFLUSH);
        ssize_t sent = write(serial_fd_, cmd, cmd_len);
        
        if (sent != cmd_len) {
            RCLCPP_ERROR(this->get_logger(), "⚠️ 指令发送失败，可能是USB断开！触发重新连接...");
            // 🔥 [核心优化] 发送失败直接关闭文件描述符，并在后台重新启动重连定时器
            close(serial_fd_);
            serial_fd_ = -1;
            if (init_timer_) {
                init_timer_->reset();
            }
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
        
        // 如果串口尚未就绪，不执行检测
        if (serial_fd_ < 0) return;

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
    rclcpp::TimerBase::SharedPtr init_timer_; // 异步重连定时器
    
    int serial_fd_;                         
    bool left_dangerous_;                   
    bool right_dangerous_;                  
    uint64_t last_left_send_ms_;            
    uint64_t last_right_send_ms_;           
    int retry_count_;
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