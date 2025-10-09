#include "serial_com_cpp/serial_communication.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>

namespace serial_com_cpp {

SerialCommunication::SerialCommunication() : pnh_("~") {
    // 获取参数
    port_ = pnh_.param<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = pnh_.param<int>("baudrate", 9600);
    timeout_ms_ = pnh_.param<int>("timeout", 500);
    
    std::string mode_str = pnh_.param<std::string>("mode", "dec");
    mode_ = (mode_str == "hex") ? DataMode::HEXADECIMAL : DataMode::DECIMAL;
    
    // 初始化串口
    if (!initializeSerial()) {
        ROS_ERROR("Failed to initialize serial port");
        return;
    }
    
    // 创建订阅者和发布者
    sub_ = nh_.subscribe("servo_serial", 10, &SerialCommunication::dataCallback, this);
    pub_ = nh_.advertise<std_msgs::String>("serial_received", 10);
    
    ROS_INFO("Serial communication initialized - Port: %s, Baudrate: %d, Mode: %s", 
             port_.c_str(), baudrate_, (mode_ == DataMode::DECIMAL) ? "decimal" : "hexadecimal");
}

SerialCommunication::~SerialCommunication() {
    if (serial_port_ && serial_port_->isOpen()) {
        serial_port_->close();
        ROS_INFO("Serial port closed");
    }
}

//  
bool SerialCommunication::initializeSerial() {
    try {
        serial_port_.reset(new serial::Serial());
        serial_port_->setPort(port_);
        serial_port_->setBaudrate(baudrate_);
        
        // 修复Timeout API使用 - 创建对象而不是临时值
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_port_->setTimeout(timeout);
        
        serial_port_->open();
        
        if (serial_port_->isOpen()) {
            ROS_INFO("Serial port opened successfully: %s at %d baud", port_.c_str(), baudrate_);
            return true;
        } else {
            ROS_ERROR("Failed to open serial port: %s", port_.c_str());
            return false;
        }
    } catch (const serial::IOException& e) {
        ROS_ERROR("Serial port exception: %s", e.what());
        return false;
    }
}

bool SerialCommunication::isConnected() const {
    return serial_port_ && serial_port_->isOpen();
}

bool SerialCommunication::sendData(const std::string& data) {
    if (!isConnected()) {
        ROS_WARN("Serial port not connected");
        return false;
    }
    
    try {
        std::vector<uint8_t> bytes;
        
        if (mode_ == DataMode::DECIMAL) {
            if (!isValidDecimal(data)) {
                ROS_WARN("Invalid decimal data: %s", data.c_str());
                return false;
            }
            
            // C++11兼容：使用atoi而不是stoi
            int value = std::atoi(data.c_str());
            if (value < 0 || value > 255) {
                ROS_WARN("Decimal value out of range (0-255): %d", value);
                return false;
            }
            bytes.push_back(static_cast<uint8_t>(value));
            
        } else { // HEXADECIMAL
            if (!isValidHexadecimal(data)) {
                ROS_WARN("Invalid hexadecimal data: %s", data.c_str());
                return false;
            }
            
            bytes = stringToBytes(data);
        }
        
        size_t bytes_written = serial_port_->write(bytes);
        
        if (bytes_written == bytes.size()) {
            ROS_INFO("Data sent successfully: %s (%zu bytes)", data.c_str(), bytes_written);
            return true;
        } else {
            ROS_WARN("Partial write: %zu/%zu bytes", bytes_written, bytes.size());
            return false;
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Error sending data: %s", e.what());
        return false;
    }
}

std::pair<bool, std::string> SerialCommunication::receiveData() {
    if (!isConnected()) {
        return std::make_pair(false, "");
    }
    
    try {
        // 检查可用数据
        size_t available = serial_port_->available();
        if (available == 0) {
            return std::make_pair(false, "");
        }
        // 读取可用数据
        std::string raw_data = serial_port_->read(available);
        std::vector<uint8_t> bytes(raw_data.begin(), raw_data.end());
        
        std::string result;
        if (mode_ == DataMode::DECIMAL) {
            int decimal_value = bytesToDecimal(bytes);
            // C++11兼容：使用stringstream而不是to_string进行数字到字符串的转化
            std::stringstream ss;
            ss << decimal_value;
            result = "Received (decimal): " + ss.str();
        } else {
            std::string hex_value = bytesToHexString(bytes);
            result = "Received (hex): " + hex_value;
        }
        
        return std::make_pair(true, result);
        
    } catch (const std::exception& e) {
        ROS_ERROR("Error receiving data: %s", e.what());
        return std::make_pair(false, "");
    }
}

void SerialCommunication::dataCallback(const std_msgs::String::ConstPtr& msg) {
    std::string data = msg->data;
    
    // 去除首尾空格
    data.erase(0, data.find_first_not_of(" \t\r\n"));
    data.erase(data.find_last_not_of(" \t\r\n") + 1);
    
    if (data.empty()) {
        ROS_WARN("Received empty data");
        return;
    }
    
    // 发送数据
    if (sendData(data)) {
        // 等待响应
        ros::Duration(0.05).sleep();
        
        auto result = receiveData();
        if (result.first) {
            std_msgs::String response_msg;
            response_msg.data = result.second;
            pub_.publish(response_msg);
        }
    }
}

std::vector<uint8_t> SerialCommunication::stringToBytes(const std::string& hex_str) {
    std::vector<uint8_t> bytes;
    std::string clean_hex = hex_str;
    
    // 移除空格和0x前缀
    clean_hex.erase(std::remove_if(clean_hex.begin(), clean_hex.end(), ::isspace), clean_hex.end());
    if (clean_hex.substr(0, 2) == "0x" || clean_hex.substr(0, 2) == "0X") {
        clean_hex = clean_hex.substr(2);
    }
    
    // 确保偶数长度
    if (clean_hex.length() % 2 != 0) {
        clean_hex = "0" + clean_hex;
    }
    
    for (size_t i = 0; i < clean_hex.length(); i += 2) {
        std::string byte_str = clean_hex.substr(i, 2);
        // C++11兼容：使用strtoul而不是stoul
        char* end;
        unsigned long val = std::strtoul(byte_str.c_str(), &end, 16);
        uint8_t byte = static_cast<uint8_t>(val);
        bytes.push_back(byte);
    }
    
    return bytes;
}

std::string SerialCommunication::bytesToHexString(const std::vector<uint8_t>& bytes) {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    
    for (uint8_t byte : bytes) {
        ss << std::setw(2) << static_cast<int>(byte);
    }
    
    return ss.str();
}

int SerialCommunication::bytesToDecimal(const std::vector<uint8_t>& bytes) {
    int result = 0;
    for (uint8_t byte : bytes) {
        result = (result << 8) | byte;
    }
    return result;
}

bool SerialCommunication::isValidDecimal(const std::string& str) {
    if (str.empty()) return false;
    
    size_t start = 0;
    if (str[0] == '+' || str[0] == '-') start = 1;
    
    for (size_t i = start; i < str.length(); ++i) {
        if (!std::isdigit(str[i])) return false;
    }
    
    return true;
}

bool SerialCommunication::isValidHexadecimal(const std::string& str) {
    if (str.empty()) return false;
    
    std::string clean_str = str;
    // 移除空格
    clean_str.erase(std::remove_if(clean_str.begin(), clean_str.end(), ::isspace), clean_str.end());
    
    // 移除0x前缀
    if (clean_str.length() >= 2 && (clean_str.substr(0, 2) == "0x" || clean_str.substr(0, 2) == "0X")) {
        clean_str = clean_str.substr(2);
    }
    
    if (clean_str.empty()) return false;
    
    for (char c : clean_str) {
        if (!std::isxdigit(c)) return false;
    }
    
    return true;
}

} // namespace serial_com_cpp
