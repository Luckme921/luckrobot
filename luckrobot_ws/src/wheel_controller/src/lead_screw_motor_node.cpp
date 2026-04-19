#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialPort.h>
#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

// 限位：0 ~ -462mm
const float MAX_LIMIT = 0.0f;
const float MIN_LIMIT = -462.0f;

const uint8_t SLAVE_ID = 1;
const std::string SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABALP8FF-if00-port0";
const LibSerial::BaudRate BAUD_RATE = LibSerial::BaudRate::BAUD_19200;
const LibSerial::Parity PARITY = LibSerial::Parity::PARITY_EVEN;
const LibSerial::StopBits STOP_BITS = LibSerial::StopBits::STOP_BITS_1;
const LibSerial::CharacterSize CHAR_SIZE = LibSerial::CharacterSize::CHAR_SIZE_8;

const float LEAD = 10.0f;
const int STEPS_PER_REV = 10000;
const int HOMING_SPEED = 20000;
const int MOVE_SPEED = 50000;

uint16_t modbus_crc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001; else crc >>= 1;
        }
    }
    return crc;
}

std::vector<uint8_t> build_write_single(uint8_t slave, uint16_t reg_addr, uint16_t value) {
    std::vector<uint8_t> frame = {slave, 0x06, (uint8_t)(reg_addr>>8), (uint8_t)reg_addr, (uint8_t)(value>>8), (uint8_t)value};
    uint16_t crc = modbus_crc16(frame);
    frame.push_back(crc&0xFF); frame.push_back((crc>>8)&0xFF); return frame;
}

std::vector<uint8_t> build_write_multiple(uint8_t slave, uint16_t reg, uint16_t cnt, const std::vector<uint16_t>& vals) {
    std::vector<uint8_t> f = {slave, 0x10, (uint8_t)(reg>>8), (uint8_t)reg, (uint8_t)(cnt>>8), (uint8_t)cnt, (uint8_t)(cnt*2)};
    for (auto v : vals) { f.push_back(v>>8); f.push_back(v&0xFF); }
    uint16_t crc = modbus_crc16(f); f.push_back(crc&0xFF); f.push_back((crc>>8)&0xFF); return f;
}

class LeadScrewMotorNode : public rclcpp::Node {
public:
    LeadScrewMotorNode() : Node("lead_screw_motor_node"), serial_(std::make_shared<LibSerial::SerialPort>()) {
        if (!init_serial()) { RCLCPP_FATAL(this->get_logger(), "串口初始化失败"); rclcpp::shutdown(); return; }

        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/displacement",
            rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
            std::bind(&LeadScrewMotorNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<std_msgs::msg::Float32>("/lead_screw/current_position", 10);
        std::thread(&LeadScrewMotorNode::homing, this).detach();

        RCLCPP_INFO(this->get_logger(), "✅ 丝杠节点(精准状态轮询版)");
        RCLCPP_INFO(this->get_logger(), "📏 限位：0 ~ -462mm");
    }

private:
    std::mutex serial_mutex_;
    std::mutex move_mutex_;

    bool init_serial() {
        try {
            serial_->Open(SERIAL_PORT);
            serial_->SetBaudRate(BAUD_RATE); serial_->SetParity(PARITY);
            serial_->SetStopBits(STOP_BITS); serial_->SetCharacterSize(CHAR_SIZE);
            serial_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true;
        } catch (...) { return false; }
    }

    // 仅发送数据的底层函数
    void send(const std::vector<uint8_t>& f) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->Write(f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 🔥 发送指令并接收返回值的函数（带超时机制）
    bool send_and_receive(const std::vector<uint8_t>& tx_frame, std::vector<uint8_t>& rx_data, size_t expected_len) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->FlushIOBuffers(); // 清空历史残留数据
        serial_->Write(tx_frame);
        
        rx_data.clear();
        auto start_time = std::chrono::steady_clock::now();
        // 500ms 超时等待数据返回
        while (rx_data.size() < expected_len) {
            if (serial_->IsDataAvailable()) {
                uint8_t byte;
                serial_->ReadByte(byte, 10);
                rx_data.push_back(byte);
            }
            if (std::chrono::steady_clock::now() - start_time > std::chrono::milliseconds(500)) {
                return false; 
            }
        }
        return true;
    }

    void homing() {
        RCLCPP_INFO(this->get_logger(), "🔧 触发回零中...");
        
        // 1. 下发回零启动序列
        std::vector<std::vector<uint8_t>> cmds = {
            build_write_single(SLAVE_ID,0x0200,9), build_write_single(SLAVE_ID,0x1003,6),
            build_write_single(SLAVE_ID,0x1023,2), build_write_multiple(SLAVE_ID,0x1024,2,{(uint16_t)HOMING_SPEED,(uint16_t)(HOMING_SPEED>>16)}),
            build_write_single(SLAVE_ID,0x0307,0), build_write_single(SLAVE_ID,0x0D08,128)
        };
        for(auto c:cmds) send(c);

        // 2. 状态轮询指令与期待的正确回复
        std::vector<uint8_t> check_status_cmd = {0x01, 0x03, 0x0B, 0x07, 0x00, 0x02, 0x77, 0xEE};
        std::vector<uint8_t> target_reply = {0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x33};
        std::vector<uint8_t> rx_buf;

        RCLCPP_INFO(this->get_logger(), "🔍 正在高频查询电机内部回零状态...");

        // 🔥 【核心优化】死循环查询状态，查到 00 00 才放行
        bool is_homing_finished = false;
        while (rclcpp::ok() && !is_homing_finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 每半秒问一次
            
            if (send_and_receive(check_status_cmd, rx_buf, 9)) {
                // 如果收到的回复与设定的目标帧完全一致
                if (rx_buf == target_reply) {
                    is_homing_finished = true;
                    RCLCPP_INFO(this->get_logger(), "🎯 收到确认回复！电机彻底回零完成！");
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "电机正在回零动作中...");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ 状态查询超时，总线可能繁忙，重试中...");
            }
        }

        // 3. 收到结束状态后，发送停止回零控制字
        RCLCPP_INFO(this->get_logger(), "⏹️ 发送解除回零状态指令...");
        send(build_write_single(SLAVE_ID, 0x0D08, 256));
        
        // 4. 重置坐标并彻底解锁移动权限
        pos_ = 0.0f; 
        home_done_.store(true);
        publish_pos();
        RCLCPP_INFO(this->get_logger(), "✅ 丝杠节点已就绪，允许接收移动指令！");
    }

    void move(float delta) {
        // 🔥 【排队挂起逻辑】：收到 ROS 话题后检查是否回零结束。
        // 如果电机还在回零，本线程会在此死循环等待，直到 homing() 把 home_done_ 设为 true。
        if (!home_done_.load()) {
            RCLCPP_INFO(this->get_logger(), "⏳ 收到移动指令(%.1f)，但回零尚未结束，已挂起等待...", delta);
            while (!home_done_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            RCLCPP_INFO(this->get_logger(), "▶️ 等待结束！立即执行堆积的移动指令(%.1f)", delta);
        }

        // 获取锁，执行正式的下降动作
        std::lock_guard<std::mutex> lock(move_mutex_);

        float curr = pos_.load();
        float target = curr + delta;
        if(target > MAX_LIMIT || target < MIN_LIMIT) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 限位保护：%.1f", target);
            return;
        }

        float revs = delta / LEAD;
        int64_t steps = revs * STEPS_PER_REV;
        uint32_t s = (uint32_t)steps;

        std::vector<std::vector<uint8_t>> cmds = {
            build_write_single(SLAVE_ID,0x0200,9), build_write_single(SLAVE_ID,0x0D11,1), build_write_single(SLAVE_ID,0x1003,1),
            build_write_multiple(SLAVE_ID,0x100E,2,{(uint16_t)s,(uint16_t)(s>>16)}),
            build_write_multiple(SLAVE_ID,0x1019,2,{(uint16_t)MOVE_SPEED,(uint16_t)(MOVE_SPEED>>16)}),
            build_write_single(SLAVE_ID,0x0D12,507), build_write_single(SLAVE_ID,0x0D08,1)
        };
        for(auto c:cmds) send(c);

        pos_ = target;
        publish_pos();

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(std::fabs(delta)*20)+500));
        send(build_write_single(SLAVE_ID,0x0D08,0));
        RCLCPP_INFO(this->get_logger(), "📌 位置: %.1f mm", pos_.load());
    }

    void callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // 创建独立线程去执行，保证就算被 move() 里的 while 挂起，也不会卡死整个 ROS 节点通信
        std::thread(&LeadScrewMotorNode::move, this, msg->data).detach();
    }

    void publish_pos() {
        std_msgs::msg::Float32 m; m.data = pos_.load(); pub_->publish(m);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    std::shared_ptr<LibSerial::SerialPort> serial_;
    std::atomic<bool> home_done_{false};
    std::atomic<float> pos_{0.0f};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeadScrewMotorNode>());
    rclcpp::shutdown();
    return 0;
}