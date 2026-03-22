#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>
#include <cmath>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <csignal>
#include <functional>
#include <string>

// ==================== 底盘硬件/协议配置（可直接修改） ====================
// 串口配置（RS485总线）
const char* SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABADW8KY-if00-port0";
const int BAUDRATE = B115200;          // 115200波特率
const int DATA_BITS = CS8;             // 8位数据位
const int STOP_BITS = 0;               // 1位停止位
const int PARITY = 0;                  // 无校验位

// 轮子参数
const double WHEEL_DIAMETER_MM = 100.0;        // 轮子直径100mm
const double WHEEL_BASE_MM = 376.0;            // 轮距376mm
const double WHEEL_CIRCUMFERENCE_CM = M_PI * (WHEEL_DIAMETER_MM / 10.0);  // 周长≈31.4159cm

// 电机ID（RS485总线）
const uint8_t LEFT_WHEEL_ID = 0x03;    // 左轮ID
const uint8_t RIGHT_WHEEL_ID = 0x04;   // 右轮ID

// 指令帧协议（严格匹配你给出的格式）
const uint8_t FUNC_CODE = 0x64;        // 功能码固定64
const uint8_t RUN_BYTE = 0x00;         // 运动位（第8字节）
const uint8_t BRAKE_BYTE = 0xFF;       // 刹车位（第8字节）
const uint8_t PLACEHOLDER_BYTE = 0x00; // 第9字节占位符
const int FRAME_LENGTH = 10;           // 指令帧长度10字节
const uint8_t CRC8_POLY_REV = 0x8C;    // CRC8/MAXIM反转多项式
const int RPM_LIMIT = 330;             // 转速限制±330rpm

// 控制参数（适配RS485一问一答）
const int CONTROL_FREQ = 100;          // 控制频率100Hz
const double CMD_VEL_TIMEOUT = 0.2;    // cmd_vel超时刹车时间（200ms）
const int RESPONSE_TIMEOUT_MS = 10;    // 电机回复超时（适配RS485总线）
const int RS485_DELAY_US = 300;        // 【修改】延长总线间隔（从200→300us，避免指令冲突）
const int BRAKE_RETRY_COUNT = 1;       // 刹车指令重试次数（减少退出延迟）
const int INIT_RETRY_COUNT = 2;        // 启动初始化重试次数

// ==================== 位移传感器配置（核心逻辑：65mm及以上=安全/无效，35~65mm=危险/有效） ====================
const uint8_t RIGHT_SENSOR_ID = 0x01;  // 右轮位移传感器ID
const uint8_t LEFT_SENSOR_ID = 0x02;   // 左轮位移传感器ID
// 传感器固定读取指令帧
const uint8_t RIGHT_SENSOR_CMD[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
const uint8_t LEFT_SENSOR_CMD[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39};
const int SENSOR_CMD_LEN = 8;          // 传感器指令帧长度
const int SENSOR_RESP_LEN = 7;         // 传感器回复帧长度
const int SENSOR_TIMEOUT_MS = 15;      // 【修改】延长传感器超时（从10→15ms，降低误判）
const double SENSOR_MIN_DANGER = 35.0; // 危险距离最小值(mm)
const double SENSOR_MAX_DANGER = 65.0; // 危险距离最大值(mm)（≥此值=安全）
const int SENSOR_SCALE = 100;          // 数据缩放（DH+DL组合值/100=实际距离mm）

// ==================== 新增：传感器/串口重试配置 【修改】 ====================
const int SENSOR_CMD_RETRY = 2;        // 传感器指令发送重试次数
const int RS485_CMD_RETRY = 2;         // RS485指令发送重试次数
const int BUFFER_CLEAR_RETRY = 3;      // 缓冲区清空重试次数

// ==================== 全局状态（安全终止） ====================
std::atomic<bool> g_node_running(false);
std::atomic<bool> g_signal_handled(false);
int g_serial_fd = -1;
std::shared_ptr<rclcpp::Logger> g_logger = nullptr;
struct sigaction g_old_sigint;

// ==================== 提前声明核心函数 ====================
static inline uint8_t crc8_maxim(const uint8_t* data, int len);
std::string hex_to_string(const uint8_t* data, int len);
void clear_serial_buffer();
bool send_rs485_command(uint8_t motor_id, int rpm, bool brake);
bool read_rs485_response(uint8_t motor_id, uint16_t& position, std::string& error_msg);
void signal_handler(int signum);
// 传感器相关函数
bool send_sensor_command(const uint8_t* cmd, int cmd_len);
bool read_sensor_response(uint8_t sensor_id, double& distance, bool& is_dangerous, std::string& error_msg);
double parse_sensor_distance(uint8_t dh, uint8_t dl);
// 新增：格式化十六进制字符串 【修改】
std::string to_hex_string(uint8_t val);

// ==================== 辅助函数：16进制打印 ====================
std::string hex_to_string(const uint8_t* data, int len) {
    if (data == nullptr || len <= 0) {
        return "空数据";
    }
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (int i = 0; i < len; ++i) {
        ss << std::setw(2) << static_cast<int>(data[i]);
        if (i < len - 1) ss << " ";
    }
    return ss.str();
}

// 【修改】新增：格式化单个字节为十六进制字符串（修复ID显示错误）
std::string to_hex_string(uint8_t val) {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(val);
    return ss.str();
}

// ==================== 辅助函数：清空串口缓冲区（增强版）【修改】 ====================
void clear_serial_buffer() {
    if (g_serial_fd < 0) return;
    tcflush(g_serial_fd, TCIOFLUSH);
    uint8_t dummy[1024];
    // 重试读取清空，确保缓冲区无残留
    for (int i = 0; i < BUFFER_CLEAR_RETRY; ++i) {
        ssize_t read_len = read(g_serial_fd, dummy, sizeof(dummy));
        if (read_len <= 0) break;
        usleep(100); // 短延时后再次读取
    }
}

// ==================== CRC8/MAXIM算法 ====================
static inline uint8_t crc8_maxim(const uint8_t* data, int len) {
    if (data == nullptr || len <= 0) return 0;
    uint8_t crc = 0x00;
    const uint8_t poly_rev = CRC8_POLY_REV;

    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ poly_rev;
            } else {
                crc = crc >> 1;
            }
            crc &= 0xFF;
        }
    }
    return crc;
}

// ==================== 信号处理（强制刹车+安全退出） ====================
void signal_handler(int signum) {
    if (g_signal_handled.load()) return;
    g_signal_handled.store(true);

    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "=== 收到终止信号(%d)，强制发送刹车指令 ===", signum);
    }
    
    sigaction(SIGINT, &g_old_sigint, nullptr);

    if (g_serial_fd >= 0) {
        uint8_t brake_frame[FRAME_LENGTH] = {0};
        brake_frame[0] = LEFT_WHEEL_ID;
        brake_frame[1] = FUNC_CODE;
        brake_frame[7] = BRAKE_BYTE;
        brake_frame[8] = PLACEHOLDER_BYTE;
        brake_frame[9] = crc8_maxim(brake_frame, 9);
        
        if (g_logger != nullptr) {
            RCLCPP_INFO(*g_logger, "发送左轮刹车指令: %s", 
                        hex_to_string(brake_frame, FRAME_LENGTH).c_str());
        }
        write(g_serial_fd, brake_frame, FRAME_LENGTH);
        tcdrain(g_serial_fd);
        usleep(RS485_DELAY_US);
        
        brake_frame[0] = RIGHT_WHEEL_ID;
        brake_frame[9] = crc8_maxim(brake_frame, 9);
        if (g_logger != nullptr) {
            RCLCPP_INFO(*g_logger, "发送右轮刹车指令: %s", 
                        hex_to_string(brake_frame, FRAME_LENGTH).c_str());
        }
        write(g_serial_fd, brake_frame, FRAME_LENGTH);
        tcdrain(g_serial_fd);
        
        if (g_logger != nullptr) {
            RCLCPP_INFO(*g_logger, "=== 刹车指令已发送到RS485总线 ===");
        }
    }
    
    g_node_running.store(false);
    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "=== 开始关闭节点 ===");
    }

    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        g_serial_fd = -1;
    }

    rclcpp::shutdown();
}

// ==================== 转速转换 ====================
static inline int cmps_to_rpm(double cm_per_s) {
    double rpm = (cm_per_s * 60.0) / WHEEL_CIRCUMFERENCE_CM;
    return static_cast<int>(round(rpm));
}

static inline std::pair<uint8_t, uint8_t> rpm_to_2bytes(int rpm) {
    rpm = std::clamp(rpm, -RPM_LIMIT, RPM_LIMIT);
    uint16_t int16_val;

    if (rpm < 0) {
        int16_val = 0x10000 + rpm;
    } else {
        int16_val = static_cast<uint16_t>(rpm);
    }

    uint8_t high_byte = static_cast<uint8_t>((int16_val >> 8) & 0xFF);
    uint8_t low_byte = static_cast<uint8_t>(int16_val & 0xFF);
    return {high_byte, low_byte};
}

// ==================== 串口初始化 ====================
bool serial_init() {
    g_serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_serial_fd < 0) {
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "串口打开失败: %s (错误码: %d)", 
                         strerror(errno), errno);
        }
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(g_serial_fd, &tty) != 0) {
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "获取串口属性失败: %s (错误码: %d)", 
                         strerror(errno), errno);
        }
        close(g_serial_fd);
        g_serial_fd = -1;
        return false;
    }

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD;
    tty.c_cflag |= CLOCAL;

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(g_serial_fd, TCSANOW, &tty) != 0) {
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "设置串口属性失败: %s (错误码: %d)", 
                         strerror(errno), errno);
        }
        close(g_serial_fd);
        g_serial_fd = -1;
        return false;
    }

    clear_serial_buffer();
    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "串口初始化成功: %s (115200 8N1 RS485)", SERIAL_PORT);
    }
    return true;
}

// ==================== RS485发送电机指令（增加重试）【修改】 ====================
bool send_rs485_command(uint8_t motor_id, int rpm, bool brake) {
    if (g_serial_fd < 0 || !g_node_running.load()) {
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "串口未初始化或节点已停止，发送失败");
        }
        return false;
    }

    uint8_t frame[FRAME_LENGTH] = {0};
    frame[0] = motor_id;
    frame[1] = FUNC_CODE;
    frame[7] = brake ? BRAKE_BYTE : RUN_BYTE;
    frame[8] = PLACEHOLDER_BYTE;

    if (!brake) {
        auto [high, low] = rpm_to_2bytes(rpm);
        frame[2] = high;
        frame[3] = low;
    }

    frame[9] = crc8_maxim(frame, 9);
    
    // 增加重试机制
    for (int retry = 0; retry < RS485_CMD_RETRY; ++retry) {
        clear_serial_buffer();

        if (g_logger != nullptr) {
            RCLCPP_INFO(*g_logger, "→ 发送电机0x%02X指令(重试%d): %s (转速: %d, 刹车: %s)",
                        motor_id, retry, hex_to_string(frame, FRAME_LENGTH).c_str(), rpm, brake ? "是" : "否");
        }
        
        ssize_t sent = write(g_serial_fd, frame, FRAME_LENGTH);
        if (sent == FRAME_LENGTH) {
            tcdrain(g_serial_fd);
            usleep(RS485_DELAY_US);
            return true;
        }

        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "← 电机0x%02X发送失败(重试%d): 发送%d字节（期望%d）",
                        motor_id, retry, static_cast<int>(sent), FRAME_LENGTH);
        }
        usleep(RS485_DELAY_US); // 重试前短延时
    }

    return false;
}

// ==================== RS485读取电机回复（修复ID显示）【修改】 ====================
bool read_rs485_response(uint8_t motor_id, uint16_t& position, std::string& error_msg) {
    position = 0;
    error_msg.clear();

    if (g_serial_fd < 0 || !g_node_running.load()) {
        error_msg = "串口未初始化或节点已停止";
        return false;
    }

    uint8_t response[FRAME_LENGTH] = {0};
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(g_serial_fd, &read_fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = RESPONSE_TIMEOUT_MS * 1000;

    int ret = select(g_serial_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        error_msg = "select错误: " + std::string(strerror(errno));
        return false;
    } else if (ret == 0) {
        error_msg = "超时(" + std::to_string(RESPONSE_TIMEOUT_MS) + "ms)，无回复";
        return false;
    }

    ssize_t recv_len = read(g_serial_fd, response, FRAME_LENGTH);
    if (recv_len != FRAME_LENGTH) {
        error_msg = "回复长度错误: 收到" + std::to_string(recv_len) + "字节（期望10）";
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "← 电机0x%02X回复: %s (长度错误)",
                         motor_id, hex_to_string(response, recv_len).c_str());
        }
        return false;
    }

    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "← 电机0x%02X原始回复: %s",
                     motor_id, hex_to_string(response, FRAME_LENGTH).c_str());
    }

    if (response[0] != motor_id) {
        // 【修改】修复ID显示为十六进制
        error_msg = "ID不匹配: 收到0x" + to_hex_string(response[0]) + "（期望0x" + to_hex_string(motor_id) + "）";
        return false;
    }

    uint8_t calc_crc = crc8_maxim(response, 9);
    if (response[9] != calc_crc) {
        // 【修改】修复CRC显示为十六进制
        error_msg = "CRC校验失败: 计算0x" + to_hex_string(calc_crc) + "（收到0x" + to_hex_string(response[9]) + "）";
        return false;
    }

    position = (static_cast<uint16_t>(response[6]) << 8) | response[7];
    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "← 电机0x%02X回复成功: 位置=0x%04X (十进制: %d, 角度: %.2f°)",
                     motor_id, position, position, (static_cast<double>(position)/32767.0)*360.0);
    }
    return true;
}

// ==================== 发送传感器读取指令（增加重试）【修改】 ====================
bool send_sensor_command(const uint8_t* cmd, int cmd_len) {
    if (g_serial_fd < 0 || !g_node_running.load() || cmd == nullptr || cmd_len <= 0) {
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "串口未初始化/节点停止/指令为空，传感器指令发送失败");
        }
        return false;
    }

    // 增加重试机制
    for (int retry = 0; retry < SENSOR_CMD_RETRY; ++retry) {
        clear_serial_buffer();

        if (g_logger != nullptr) {
            RCLCPP_INFO(*g_logger, "→ 发送传感器指令(重试%d): %s", retry, hex_to_string(cmd, cmd_len).c_str());
        }
        
        ssize_t sent = write(g_serial_fd, cmd, cmd_len);
        if (sent == cmd_len) {
            tcdrain(g_serial_fd);
            usleep(RS485_DELAY_US);
            return true;
        }

        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "传感器指令发送失败(重试%d): 发送%d字节（期望%d）",
                        retry, static_cast<int>(sent), cmd_len);
        }
        usleep(RS485_DELAY_US); // 重试前短延时
    }

    return false;
}

// ==================== 解析传感器距离值 ====================
double parse_sensor_distance(uint8_t dh, uint8_t dl) {
    uint16_t raw_value = (static_cast<uint16_t>(dh) << 8) | dl;
    return static_cast<double>(raw_value) / SENSOR_SCALE;
}

// ==================== 读取传感器回复（修复ID显示）【修改】 ====================
bool read_sensor_response(uint8_t sensor_id, double& distance, bool& is_dangerous, std::string& error_msg) {
    distance = 0.0;
    is_dangerous = false;
    error_msg.clear();

    if (g_serial_fd < 0 || !g_node_running.load()) {
        error_msg = "串口未初始化或节点已停止";
        return false;
    }

    uint8_t response[SENSOR_RESP_LEN] = {0};
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(g_serial_fd, &read_fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = SENSOR_TIMEOUT_MS * 1000;

    int ret = select(g_serial_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        error_msg = "select错误: " + std::string(strerror(errno));
        return false;
    } else if (ret == 0) {
        error_msg = "传感器超时(" + std::to_string(SENSOR_TIMEOUT_MS) + "ms)，无回复";
        return false;
    }

    ssize_t recv_len = read(g_serial_fd, response, SENSOR_RESP_LEN);
    if (recv_len != SENSOR_RESP_LEN) {
        error_msg = "传感器回复长度错误: 收到" + std::to_string(recv_len) + "字节（期望7）";
        if (g_logger != nullptr) {
            RCLCPP_ERROR(*g_logger, "← 传感器0x%02X回复: %s (长度错误)",
                         sensor_id, hex_to_string(response, recv_len).c_str());
        }
        return false;
    }

    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "← 传感器0x%02X原始回复: %s",
                     sensor_id, hex_to_string(response, SENSOR_RESP_LEN).c_str());
    }

    if (response[0] != sensor_id) {
        // 【修改】修复ID显示为十六进制
        error_msg = "传感器ID不匹配: 收到0x" + to_hex_string(response[0]) + "（期望0x" + to_hex_string(sensor_id) + "）";
        return false;
    }

    if (response[1] != 0x03) {
        // 【修改】修复功能码显示为十六进制
        error_msg = "传感器功能码错误: 收到0x" + to_hex_string(response[1]) + "（期望0x03）";
        return false;
    }

    if (response[2] != 0x02) {
        // 【修改】修复数据长度显示为十六进制
        error_msg = "传感器数据长度错误: 收到0x" + to_hex_string(response[2]) + "（期望0x02）";
        return false;
    }

    // 核心判断：35≤距离<65 → 危险（需要避障）；≥65 → 安全（无风险）
    distance = parse_sensor_distance(response[3], response[4]);
    is_dangerous = (distance >= SENSOR_MIN_DANGER) && (distance < SENSOR_MAX_DANGER);

    if (g_logger != nullptr) {
        RCLCPP_INFO(*g_logger, "← 传感器0x%02X解析成功: 原始值=0x%02X%02X | 距离=%.2fmm | 状态: %s",
                     sensor_id, response[3], response[4], distance, 
                     is_dangerous ? "危险（<65mm）" : "安全（≥65mm）");
    }

    return true;
}

// ==================== 运动学解算 ====================
static inline void calculate_wheel_rpm(double linear_x, double angular_z, int& left_rpm, int& right_rpm) {
    double linear_cm = linear_x * 100.0;
    double angular_cm = angular_z * (WHEEL_BASE_MM / 20.0);

    double left_vel_cm = linear_cm + angular_cm;
    double right_vel_cm = linear_cm - angular_cm;
    right_vel_cm = -right_vel_cm;

    left_rpm = cmps_to_rpm(left_vel_cm);
    right_rpm = cmps_to_rpm(right_vel_cm);
}

// ==================== 核心节点（新增传感器历史值）【修改】 ====================
class WheelControllerNode : public rclcpp::Node {
public:
    WheelControllerNode() : Node("wheel_controller"),
                            linear_x_(0.0),
                            angular_z_(0.0),
                            brake_sent_(false),
                            last_valid_left_pos_(0),
                            last_valid_right_pos_(0),
                            publisher_ready_(false),
                            // 【修改】初始化传感器历史值
                            last_left_sensor_dist_(0.0),
                            last_right_sensor_dist_(0.0),
                            left_sensor_first_valid_(false),
                            right_sensor_first_valid_(false)
    {
        g_logger = std::make_shared<rclcpp::Logger>(this->get_logger());
        g_node_running.store(true);
        g_signal_handled.store(false);

        create_publishers_and_subscribers();

        if (!serial_init()) {
            RCLCPP_FATAL(this->get_logger(), "RS485串口初始化失败，节点退出");
            g_node_running.store(false);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "=== 启动初始化：发送速度0指令获取初始位置 ===");
        if (!init_motor_position()) {
            RCLCPP_WARN(this->get_logger(), "=== 启动初始化失败，使用默认位置0 ===");
        } else {
            RCLCPP_INFO(this->get_logger(), "=== 启动初始化成功 | 左轮初始位置: %d(%.2f°) | 右轮初始位置: %d(%.2f°) ===",
                         last_valid_left_pos_, (static_cast<double>(last_valid_left_pos_)/32767.0)*360.0,
                         last_valid_right_pos_, (static_cast<double>(last_valid_right_pos_)/32767.0)*360.0);
            publish_wheel_position(last_valid_left_pos_, last_valid_right_pos_, "启动初始化位置");
        }

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&WheelControllerNode::control_loop, this));

        last_cmd_vel_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "=== 底盘控制节点启动成功（RS485一问一答模式） ===");
        RCLCPP_INFO(this->get_logger(), "控制频率: %dHz | 超时刹车: %.1fms | RS485超时: %dms | 总线间隔: %dus",
                    CONTROL_FREQ, CMD_VEL_TIMEOUT * 1000, RESPONSE_TIMEOUT_MS, RS485_DELAY_US);
        RCLCPP_INFO(this->get_logger(), "=== 位移传感器配置：35~65mm=危险 | ≥65mm=安全 | 解析缩放×%d ===", SENSOR_SCALE);
    }

    ~WheelControllerNode() {
        if (control_timer_) {
            control_timer_->cancel();
        }

        RCLCPP_INFO(this->get_logger(), "=== 析构函数：发送最终刹车指令 ===");
        
        if (g_serial_fd >= 0 && g_node_running.load()) {
            send_rs485_command(LEFT_WHEEL_ID, 0, true);
            send_rs485_command(RIGHT_WHEEL_ID, 0, true);
        }

        if (g_serial_fd >= 0) {
            close(g_serial_fd);
            g_serial_fd = -1;
        }

        RCLCPP_INFO(this->get_logger(), "=== 节点已关闭，电机已刹车 ===");
        g_node_running.store(false);
    }

private:
    void create_publishers_and_subscribers() {
        // 轮子位置发布器
        wheel_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "wheel_position", 10);

        // 传感器距离发布器（单个）
        left_sensor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "left_obstacle_distance", 10);
        right_sensor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "right_obstacle_distance", 10);
        
        // 传感器组合数据发布器（距离+危险状态）
        sensor_dist_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "obstacle_distances", 10);

        // 速度订阅器
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&WheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));

        publisher_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "所有发布器/订阅器创建成功");
    }

    bool init_motor_position() {
        if (g_serial_fd < 0 || !g_node_running.load()) {
            RCLCPP_ERROR(this->get_logger(), "串口未初始化或节点已停止，无法执行启动初始化");
            return false;
        }

        uint16_t left_pos = 0, right_pos = 0;
        std::string left_err, right_err;

        for (int i = 0; i < INIT_RETRY_COUNT && g_node_running.load(); ++i) {
            if (send_rs485_command(LEFT_WHEEL_ID, 0, false)) {
                if (read_rs485_response(LEFT_WHEEL_ID, left_pos, left_err)) {
                    last_valid_left_pos_ = left_pos;
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "左轮初始化重试(%d/%d): %s", i+1, INIT_RETRY_COUNT, left_err.c_str());
            }
            usleep(RS485_DELAY_US);
        }

        usleep(RS485_DELAY_US);
        for (int i = 0; i < INIT_RETRY_COUNT && g_node_running.load(); ++i) {
            if (send_rs485_command(RIGHT_WHEEL_ID, 0, false)) {
                if (read_rs485_response(RIGHT_WHEEL_ID, right_pos, right_err)) {
                    last_valid_right_pos_ = right_pos;
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "右轮初始化重试(%d/%d): %s", i+1, INIT_RETRY_COUNT, right_err.c_str());
            }
            usleep(RS485_DELAY_US);
        }

        return (left_pos != 0) || (right_pos != 0);
    }

    // 发布传感器数据（危险状态：1=危险，0=安全）
    void publish_sensor_data(double left_dist, bool left_dangerous, double right_dist, bool right_dangerous) {
        if (!publisher_ready_ || !g_node_running.load()) {
            RCLCPP_WARN(this->get_logger(), "发布器未就绪或节点已停止，跳过传感器数据发布");
            return;
        }

        // 发布左传感器距离
        std_msgs::msg::Float64 left_msg;
        left_msg.data = left_dist;
        left_sensor_pub_->publish(left_msg);

        // 发布右传感器距离
        std_msgs::msg::Float64 right_msg;
        right_msg.data = right_dist;
        right_sensor_pub_->publish(right_msg);

        // 发布组合数据：[左距离, 右距离, 左危险状态(1/0), 右危险状态(1/0)]
        std_msgs::msg::Float64MultiArray dist_msg;
        dist_msg.data.resize(4);
        dist_msg.data[0] = left_dist;
        dist_msg.data[1] = right_dist;
        dist_msg.data[2] = left_dangerous ? 1.0 : 0.0;
        dist_msg.data[3] = right_dangerous ? 1.0 : 0.0;
        sensor_dist_pub_->publish(dist_msg);

        RCLCPP_INFO(this->get_logger(), "[sensor] === 发布传感器数据 | 左距离: %.2fmm(%s) | 右距离: %.2fmm(%s) ===",
                     left_dist, left_dangerous ? "危险" : "安全",
                     right_dist, right_dangerous ? "危险" : "安全");
    }

    // 读取传感器数据（核心修改：失败时沿用历史值）【修改】
    void read_sensor_data(double& left_dist, bool& left_dangerous, double& right_dist, bool& right_dangerous) {
        left_dist = 0.0;
        left_dangerous = false;
        right_dist = 0.0;
        right_dangerous = false;
        std::string left_err, right_err;

        // 读取右传感器
        bool right_success = false;
        if (send_sensor_command(RIGHT_SENSOR_CMD, SENSOR_CMD_LEN)) {
            right_success = read_sensor_response(RIGHT_SENSOR_ID, right_dist, right_dangerous, right_err);
        } else {
            right_err = "右传感器指令发送失败";
        }

        // 【修改】右传感器读取失败：沿用历史值
        if (!right_success && !right_err.empty()) {
            right_dist = last_right_sensor_dist_;
            // 首次失败时危险状态设为false（安全）
            right_dangerous = right_sensor_first_valid_ ? (right_dist >= SENSOR_MIN_DANGER && right_dist < SENSOR_MAX_DANGER) : false;
            RCLCPP_WARN(this->get_logger(), "右传感器读取失败，沿用历史值: %.2fmm | 错误: %s", right_dist, right_err.c_str());
        } else {
            // 读取成功：更新历史值
            last_right_sensor_dist_ = right_dist;
            right_sensor_first_valid_ = true;
        }

        usleep(RS485_DELAY_US);

        // 读取左传感器
        bool left_success = false;
        if (send_sensor_command(LEFT_SENSOR_CMD, SENSOR_CMD_LEN)) {
            left_success = read_sensor_response(LEFT_SENSOR_ID, left_dist, left_dangerous, left_err);
        } else {
            left_err = "左传感器指令发送失败";
        }

        // 【修改】左传感器读取失败：沿用历史值
        if (!left_success && !left_err.empty()) {
            left_dist = last_left_sensor_dist_;
            // 首次失败时危险状态设为false（安全）
            left_dangerous = left_sensor_first_valid_ ? (left_dist >= SENSOR_MIN_DANGER && left_dist < SENSOR_MAX_DANGER) : false;
            RCLCPP_WARN(this->get_logger(), "左传感器读取失败，沿用历史值: %.2fmm | 错误: %s", left_dist, left_err.c_str());
        } else {
            // 读取成功：更新历史值
            last_left_sensor_dist_ = left_dist;
            left_sensor_first_valid_ = true;
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!g_node_running.load() || msg == nullptr) return;

        std::lock_guard<std::mutex> lock(vel_mutex_);
        linear_x_ = msg->linear.x;
        angular_z_ = msg->angular.z;
        last_cmd_vel_time_ = this->now();
        brake_sent_ = false;
        RCLCPP_DEBUG(this->get_logger(), "收到cmd_vel: linear.x=%.2f m/s, angular.z=%.2f rad/s",
                     linear_x_, angular_z_);
    }

    void control_loop() {
        if (!g_node_running.load() || !rclcpp::ok()) return;

        // 1. 100Hz读取传感器数据并发布
        double left_sensor_dist = 0.0, right_sensor_dist = 0.0;
        bool left_dangerous = false, right_dangerous = false;
        read_sensor_data(left_sensor_dist, left_dangerous, right_sensor_dist, right_dangerous);
        publish_sensor_data(left_sensor_dist, left_dangerous, right_sensor_dist, right_dangerous);

        // 2. 原有电机控制逻辑（无任何修改）
        double time_since_last_cmd = (this->now() - last_cmd_vel_time_).seconds();
        if (time_since_last_cmd > CMD_VEL_TIMEOUT) {
            if (!brake_sent_) {
                RCLCPP_WARN(this->get_logger(), "=== 超时%.1fms，发送刹车指令 ===",
                            time_since_last_cmd * 1000);
                send_rs485_command(LEFT_WHEEL_ID, 0, true);
                send_rs485_command(RIGHT_WHEEL_ID, 0, true);
                brake_sent_ = true;
            }
            publish_wheel_position(last_valid_left_pos_, last_valid_right_pos_, 
                                  "超时刹车，使用最后有效位置 | 左轮最后有效: " + std::to_string(last_valid_left_pos_) + 
                                  " | 右轮最后有效: " + std::to_string(last_valid_right_pos_));
            return;
        }

        int left_rpm, right_rpm;
        {
            std::lock_guard<std::mutex> lock(vel_mutex_);
            calculate_wheel_rpm(linear_x_, angular_z_, left_rpm, right_rpm);
        }

        uint16_t left_pos = 0, right_pos = 0;
        std::string left_err, right_err;

        if (send_rs485_command(LEFT_WHEEL_ID, left_rpm, false)) {
            if (read_rs485_response(LEFT_WHEEL_ID, left_pos, left_err)) {
                last_valid_left_pos_ = left_pos;
            }
        } else {
            left_err = "指令发送失败";
        }

        usleep(RS485_DELAY_US);
        if (send_rs485_command(RIGHT_WHEEL_ID, right_rpm, false)) {
            if (read_rs485_response(RIGHT_WHEEL_ID, right_pos, right_err)) {
                last_valid_right_pos_ = right_pos;
            }
        } else {
            right_err = "指令发送失败";
        }

        std::string pub_note = "实时读取 | 左轮: " + (left_err.empty() ? "成功" : left_err) + 
                               " | 右轮: " + (right_err.empty() ? "成功" : right_err) +
                               " | 左轮最后有效: " + std::to_string(last_valid_left_pos_) +
                               " | 右轮最后有效: " + std::to_string(last_valid_right_pos_);
        publish_wheel_position(left_pos, right_pos, pub_note);
    }

    void publish_wheel_position(uint16_t left_pos, uint16_t right_pos, const std::string& note) {
        if (!publisher_ready_ || wheel_pos_pub_ == nullptr || !g_node_running.load()) {
            RCLCPP_WARN(this->get_logger(), "发布器未就绪或节点已停止，跳过位置发布");
            return;
        }

        std_msgs::msg::Float64MultiArray pos_msg;
        pos_msg.data.resize(2);

        double left_angle = (static_cast<double>(left_pos) / 32767.0) * 360.0;
        double right_angle = (static_cast<double>(right_pos) / 32767.0) * 360.0;

        pos_msg.data[0] = left_angle;
        pos_msg.data[1] = right_angle;

        RCLCPP_INFO(this->get_logger(), "[position] === 发布位置 | 左轮实时: %d(%.2f°) | 右轮实时: %d(%.2f°) | 备注: %s ===",
                     left_pos, left_angle, right_pos, right_angle, note.c_str());
        wheel_pos_pub_->publish(pos_msg);
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pos_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 传感器发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_sensor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_sensor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_dist_pub_;

    double linear_x_;
    double angular_z_;
    rclcpp::Time last_cmd_vel_time_;
    bool brake_sent_;
    std::mutex vel_mutex_;
    
    uint16_t last_valid_left_pos_;
    uint16_t last_valid_right_pos_;
    bool publisher_ready_;

    // 【修改】新增：传感器历史值存储
    double last_left_sensor_dist_;
    double last_right_sensor_dist_;
    bool left_sensor_first_valid_;   // 标记左传感器是否有过有效读数
    bool right_sensor_first_valid_;  // 标记右传感器是否有过有效读数
};

// ==================== 主函数 ====================
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, &g_old_sigint);

    auto node = std::make_shared<WheelControllerNode>();

    if (g_node_running.load()) {
        rclcpp::spin(node);
    }

    sigaction(SIGINT, &g_old_sigint, nullptr);
    node.reset();
    rclcpp::shutdown();
    
    return 0;
}