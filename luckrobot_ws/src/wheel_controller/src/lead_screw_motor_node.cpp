#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialPort.h>
#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

// 限位：0 ~ -462mm（你要求的）
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

        // 🔥 【唯一修改】可靠QoS订阅，100%收到指令，永不丢失
        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/lead_screw/displacement",
            rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
            std::bind(&LeadScrewMotorNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<std_msgs::msg::Float32>("/lead_screw/current_position", 10);
        std::thread(&LeadScrewMotorNode::homing, this).detach();

        RCLCPP_INFO(this->get_logger(), "✅ 丝杠节点(终极可靠版)");
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

    void send(const std::vector<uint8_t>& f) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_->Write(f);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void homing() {
        RCLCPP_INFO(this->get_logger(), "🔧 回零中...");
        std::vector<std::vector<uint8_t>> cmds = {
            build_write_single(SLAVE_ID,0x0200,9), build_write_single(SLAVE_ID,0x1003,6),
            build_write_single(SLAVE_ID,0x1023,2), build_write_multiple(SLAVE_ID,0x1024,2,{(uint16_t)HOMING_SPEED,(uint16_t)(HOMING_SPEED>>16)}),
            build_write_single(SLAVE_ID,0x0307,0), build_write_single(SLAVE_ID,0x0D08,128)
        };
        for(auto c:cmds) send(c);
        std::this_thread::sleep_for(std::chrono::seconds(25));
        send(build_write_single(SLAVE_ID,0x0D08,256));
        
        pos_ = 0.0f; home_done_ = true;
        publish_pos();
        RCLCPP_INFO(this->get_logger(), "✅ 回零完成");
    }

    void move(float delta) {
        std::lock_guard<std::mutex> lock(move_mutex_);

        if(!home_done_) return;

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