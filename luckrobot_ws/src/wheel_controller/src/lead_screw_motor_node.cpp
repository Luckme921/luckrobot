#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialPort.h>
#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <atomic>

// Modbus RTU 相关配置
const uint8_t SLAVE_ID = 1;
const std::string SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABALP8FF-if00-port0";
// 波特率枚举（19200对应BAUD_19200）
const LibSerial::BaudRate BAUD_RATE = LibSerial::BaudRate::BAUD_19200;
// 最终修正：匹配LibSerial实际枚举名
const LibSerial::Parity PARITY = LibSerial::Parity::PARITY_EVEN;
const LibSerial::StopBits STOP_BITS = LibSerial::StopBits::STOP_BITS_1;
const LibSerial::CharacterSize CHAR_SIZE = LibSerial::CharacterSize::CHAR_SIZE_8;  // 关键修正：CHAR_SIZE_8

// 丝杆参数（和Python一致）
const float LEAD = 10.0f;          // 导程mm/圈
const int STEPS_PER_REV = 10000;   // 细分数
const int HOMING_SPEED = 20000;    // 回零速度Pr/s
const int MOVE_SPEED = 50000;      // 移动速度Pr/s

/**
 * @brief 计算Modbus RTU CRC16校验码
 * @param data 待校验的字节数组
 * @return 16位CRC值（低字节在前）
 */
uint16_t modbus_crc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 构建Modbus写单个寄存器指令（0x06功能码）
 * @param slave 从站ID
 * @param reg_addr 寄存器地址（16位）
 * @param value 寄存器值（16位）
 * @return 完整的Modbus指令帧
 */
std::vector<uint8_t> build_write_single(uint8_t slave, uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> frame;
    frame.push_back(slave);
    frame.push_back(0x06);  // 写单个寄存器功能码
    
    // 寄存器地址（大端）
    frame.push_back((reg_addr >> 8) & 0xFF);
    frame.push_back(reg_addr & 0xFF);
    
    // 寄存器值（大端）
    frame.push_back((value >> 8) & 0xFF);
    frame.push_back(value & 0xFF);
    
    // CRC校验（低字节在前）
    uint16_t crc = modbus_crc16(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    
    return frame;
}

/**
 * @brief 构建Modbus写多个寄存器指令（0x10功能码）
 * @param slave 从站ID
 * @param start_reg 起始寄存器地址
 * @param reg_count 寄存器数量
 * @param values 寄存器值列表（16位）
 * @return 完整的Modbus指令帧
 */
std::vector<uint8_t> build_write_multiple(uint8_t slave, uint16_t start_reg, uint16_t reg_count, 
                                         const std::vector<uint16_t>& values) {
    std::vector<uint8_t> frame;
    frame.push_back(slave);
    frame.push_back(0x10);  // 写多个寄存器功能码
    
    // 起始地址（大端）
    frame.push_back((start_reg >> 8) & 0xFF);
    frame.push_back(start_reg & 0xFF);
    
    // 寄存器数量（大端）
    frame.push_back((reg_count >> 8) & 0xFF);
    frame.push_back(reg_count & 0xFF);
    
    // 字节数
    frame.push_back(reg_count * 2);
    
    // 寄存器值（大端）
    for (uint16_t val : values) {
        frame.push_back((val >> 8) & 0xFF);
        frame.push_back(val & 0xFF);
    }
    
    // CRC校验
    uint16_t crc = modbus_crc16(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    
    return frame;
}

class LeadScrewMotorNode : public rclcpp::Node {
public:
    // 修正：调整成员初始化顺序，解决警告
    LeadScrewMotorNode() : Node("lead_screw_motor_node"), 
                           serial_port_(std::make_shared<LibSerial::SerialPort>()),
                           homing_completed_(false) {
        // 1. 初始化串口
        if (!init_serial()) {
            RCLCPP_FATAL(this->get_logger(), "串口初始化失败，节点退出！");
            rclcpp::shutdown();
            return;
        }

        // 2. 订阅位移指令话题（ROS2标准Float32类型）
        displacement_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/displacement", 10,
            std::bind(&LeadScrewMotorNode::displacement_callback, this, std::placeholders::_1)
        );

        // 3. 启动自动回零流程（异步执行，避免阻塞spin）
        homing_thread_ = std::thread(&LeadScrewMotorNode::homing_motor, this);

        RCLCPP_INFO(this->get_logger(), "丝杠滑台电机节点启动成功！");
        RCLCPP_INFO(this->get_logger(), "等待回零完成后，可发布 /lead_screw/displacement 话题控制位移（单位：mm）");
    }

    ~LeadScrewMotorNode() {
        // 资源释放
        if (homing_thread_.joinable()) {
            homing_thread_.join();
        }
        if (serial_port_->IsOpen()) {
            serial_port_->Close();
            RCLCPP_INFO(this->get_logger(), "串口已关闭");
        }
    }

private:
    /**
     * @brief 初始化串口
     * @return 成功返回true，失败返回false
     */
    bool init_serial() {
        try {
            serial_port_->Open(SERIAL_PORT);
            // 直接使用枚举类波特率值
            serial_port_->SetBaudRate(BAUD_RATE);
            serial_port_->SetParity(PARITY);
            serial_port_->SetStopBits(STOP_BITS);
            serial_port_->SetCharacterSize(CHAR_SIZE);
            // 流控枚举（完整路径）
            serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            
            // 串口打开后延时稳定
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            RCLCPP_INFO(this->get_logger(), "串口初始化成功：%s", SERIAL_PORT.c_str());
            return true;
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_ERROR(this->get_logger(), "串口打开失败：%s", e.what());
        } catch (const LibSerial::NotOpen& e) {
            RCLCPP_ERROR(this->get_logger(), "串口未打开：%s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "串口初始化异常：%s", e.what());
        }
        return false;
    }

    /**
     * @brief 发送Modbus指令并打印日志
     * @param frame 指令帧
     * @param cmd_desc 指令描述
     */
    void send_modbus_cmd(const std::vector<uint8_t>& frame, const std::string& cmd_desc) {
        try {
            if (!serial_port_->IsOpen()) {
                RCLCPP_ERROR(this->get_logger(), "串口未打开，无法发送指令：%s", cmd_desc.c_str());
                return;
            }

            // 发送指令
            serial_port_->Write(frame);
            
            // 打印16进制日志
            std::stringstream ss;
            for (uint8_t byte : frame) {
                ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "发送指令 [%s]：%s", cmd_desc.c_str(), ss.str().c_str());
            
            // 指令间隔延时
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送指令失败 [%s]：%s", cmd_desc.c_str(), e.what());
        }
    }

    /**
     * @brief 自动回零流程（异步线程执行）
     */
    void homing_motor() {
        RCLCPP_INFO(this->get_logger(), "=====================================");
        RCLCPP_INFO(this->get_logger(), "开始自动回零流程...");
        RCLCPP_INFO(this->get_logger(), "=====================================");

        try {
            // 拆分32位回零速度
            uint16_t low16_home = HOMING_SPEED & 0xFFFF;
            uint16_t high16_home = (HOMING_SPEED >> 16) & 0xFFFF;

            // 回零指令序列（和Python一致）
            std::vector<std::pair<std::vector<uint8_t>, std::string>> homing_cmds = {
                {build_write_single(SLAVE_ID, 0x0200, 9), "设置Modbus总线模式"},
                {build_write_single(SLAVE_ID, 0x1003, 6), "切换为Home模式"},
                {build_write_single(SLAVE_ID, 0x1023, 2), "设置正向回原模式"},
                {build_write_multiple(SLAVE_ID, 0x1024, 2, {low16_home, high16_home}), "设置回零速度"},
                {build_write_single(SLAVE_ID, 0x0307, 0), "伺服上电使能"},
                {build_write_single(SLAVE_ID, 0x0D08, 128), "启动Home回零"}
            };

            // 发送回零指令
            for (auto& [cmd, desc] : homing_cmds) {
                send_modbus_cmd(cmd, desc);
            }

            // 等待回零完成（25秒，和Python一致）
            RCLCPP_INFO(this->get_logger(), "等待光电传感器触发回零到位...（预计25秒）");
            std::this_thread::sleep_for(std::chrono::seconds(25));

            // 发送停止指令
            auto stop_cmd = build_write_single(SLAVE_ID, 0x0D08, 256);
            send_modbus_cmd(stop_cmd, "回零停止");

            // 标记回零完成
            homing_completed_ = true;
            RCLCPP_INFO(this->get_logger(), "=====================================");
            RCLCPP_INFO(this->get_logger(), "自动回零完成！可接收位移指令");
            RCLCPP_INFO(this->get_logger(), "=====================================");

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "回零流程失败：%s", e.what());
            homing_completed_ = false;
        }
    }

    /**
     * @brief 位移控制核心逻辑
     * @param displacement_mm 目标位移（mm）
     */
    void move_motor(float displacement_mm) {
        if (!homing_completed_) {
            RCLCPP_WARN(this->get_logger(), "回零未完成，拒绝执行位移指令！");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "开始执行位移控制：%.2f mm", displacement_mm);
        
        // 计算转动圈数和目标步数
        float revs = displacement_mm / LEAD;
        int64_t target_steps = static_cast<int64_t>(revs * STEPS_PER_REV);
        RCLCPP_INFO(this->get_logger(), "目标位移：%.2f mm → 转动圈数：%.2f → 目标步数：%ld", 
                    displacement_mm, revs, target_steps);

        // 拆分32位步数（处理负数）
        uint32_t target_steps_32 = static_cast<uint32_t>(target_steps);
        uint16_t low16_steps = target_steps_32 & 0xFFFF;
        uint16_t high16_steps = (target_steps_32 >> 16) & 0xFFFF;

        // 拆分速度
        uint16_t low16_speed = MOVE_SPEED & 0xFFFF;
        uint16_t high16_speed = (MOVE_SPEED >> 16) & 0xFFFF;

        try {
            // 位移指令序列（和Python一致）
            std::vector<std::pair<std::vector<uint8_t>, std::string>> move_cmds = {
                {build_write_single(SLAVE_ID, 0x0200, 9), "确保Modbus总线模式"},
                {build_write_single(SLAVE_ID, 0x0D11, 1), "强制DI使能"},
                {build_write_single(SLAVE_ID, 0x1003, 1), "切换为PP运行模式"},
                {build_write_multiple(SLAVE_ID, 0x100E, 2, {low16_steps, high16_steps}), "设置目标位置"},
                {build_write_multiple(SLAVE_ID, 0x1019, 2, {low16_speed, high16_speed}), "设置移动速度"},
                {build_write_single(SLAVE_ID, 0x0D12, 507), "电机使能"},
                {build_write_single(SLAVE_ID, 0x0D08, 1), "启动相对定位"}
            };

            // 发送位移指令
            for (auto& [cmd, desc] : move_cmds) {
                send_modbus_cmd(cmd, desc);
            }

            // 计算等待时间（优化：按速度动态计算）
            float move_time_original = fabs(revs);
            float move_time_optimized = move_time_original / 5.0f;
            float wait_time = move_time_optimized + 1.0f;
            RCLCPP_INFO(this->get_logger(), "预计移动时间：%.2f 秒（原60r/min需%.2f秒），等待中...", 
                        move_time_optimized, move_time_original);
            
            std::this_thread::sleep_for(std::chrono::duration<float>(wait_time));

            // 复位指令
            auto reset_cmd = build_write_single(SLAVE_ID, 0x0D08, 0);
            send_modbus_cmd(reset_cmd, "复位指令");

            RCLCPP_INFO(this->get_logger(), "位移控制执行完成！");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "位移控制失败：%s", e.what());
        }
    }

    /**
     * @brief 位移指令话题回调函数
     * @param msg 位移指令消息（std_msgs/msg/Float32）
     */
    void displacement_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        float displacement = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到位移指令：%.2f mm", displacement);
        move_motor(displacement);
    }

    // 成员变量（按初始化顺序排列）
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr displacement_sub_;
    std::shared_ptr<LibSerial::SerialPort> serial_port_;
    std::thread homing_thread_;
    std::atomic<bool> homing_completed_;  // 回零完成标志（原子变量保证线程安全）
};

int main(int argc, char * argv[]) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<LeadScrewMotorNode>();
    
    // 运行节点
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}
//ros2 topic pub /lead_screw/displacement std_msgs/msg/Float32 "{data: -10.0}" --once