#include "serial_com_cpp/underwater_serial_communication.h"
#include <sstream>
#include <iomanip>

namespace underwater_serial {

// 在 .cpp 文件末尾添加以下定义
const uint8_t underwater_serial::UnderwaterSerialCommunication::START_BYTE_1;
const uint8_t underwater_serial::UnderwaterSerialCommunication::START_BYTE_2;
const uint8_t underwater_serial::UnderwaterSerialCommunication::END_BYTE_1;
const uint8_t underwater_serial::UnderwaterSerialCommunication::END_BYTE_2;

UnderwaterSerialCommunication::UnderwaterSerialCommunication() 
    : pnh_("~"), should_stop_(false), rx_state_(ReceiveState::WAIT_START_1),
      rx_data_type_(0), rx_data_length_(0), rx_crc_(0), rx_data_count_(0) {
    
    // 获取参数
    pnh_.param<std::string>("port", port_, "/dev/stm32");
    pnh_.param<int>("baudrate", baudrate_, 9600);
    pnh_.param<double>("timeout", timeout_, 0.5);
    
    // 初始化CRC8查找表
    generateCRC8Table();
    
    // 初始化串口
    if (!initializeSerial()) {
        ROS_ERROR("Failed to initialize serial port");
        return;
    }
    
    // 创建订阅者和发布者
    thruster_sub_ = nh_.subscribe("thrusters_pwm", 10, 
        &UnderwaterSerialCommunication::thrusterCallback, this);
    servo_sub_ = nh_.subscribe("servo_pwm", 10,
        &UnderwaterSerialCommunication::servoCallback, this);
    sensor_pub_ = nh_.advertise<underwater_msgs::SensorData>("sensor_data", 10);
    
    // 重置状态机
    resetStateMachine();
    
    ROS_INFO("水下机器人串口通信节点已启动");
    
    // 启动接收线程
    receive_thread_ = std::thread(&UnderwaterSerialCommunication::receiveLoop, this);
}

UnderwaterSerialCommunication::~UnderwaterSerialCommunication() {
    close();
}

bool UnderwaterSerialCommunication::initializeSerial() {
    try {
        //serial_port_ = std::make_unique<serial::Serial>(port_, baudrate_, 
        //    serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ * 1000)));
        serial_port_.reset(new serial::Serial(port_, baudrate_,
                                     serial::Timeout::simpleTimeout(1000)));
        
        if (serial_port_->isOpen()) {
            ROS_INFO("串口打开成功: %s, 波特率: %d", port_.c_str(), baudrate_);
            return true;
        } else {
            ROS_ERROR("串口打开失败");
            return false;
        }
    } catch (const serial::IOException& e) {
        ROS_ERROR("串口打开失败: %s", e.what());
        return false;
    }
}

void UnderwaterSerialCommunication::generateCRC8Table() {
    crc8_table_.resize(256);
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
        crc8_table_[i] = crc;
    }
}

uint8_t UnderwaterSerialCommunication::calculateCRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (uint8_t byte : data) {
        crc = crc8_table_[crc ^ byte];
    }
    return crc;
}

std::vector<uint8_t> UnderwaterSerialCommunication::encodeData(uint8_t data_type, 
                                                               const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    
    // 添加起始字节
    frame.push_back(START_BYTE_1);
    frame.push_back(START_BYTE_2);
    
    // 添加数据类型和长度
    frame.push_back(data_type);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    
    // 添加数据负载
    frame.insert(frame.end(), payload.begin(), payload.end());
    
    // 计算并添加CRC
    std::vector<uint8_t> crc_data = {data_type, static_cast<uint8_t>(payload.size())};
    crc_data.insert(crc_data.end(), payload.begin(), payload.end());
    uint8_t crc = calculateCRC8(crc_data);
    frame.push_back(crc);
    
    // 添加结束字节
    frame.push_back(END_BYTE_1);
    frame.push_back(END_BYTE_2);
    
    return frame;
}

bool UnderwaterSerialCommunication::sendThrusterData(const std::vector<int16_t>& pwm_values) {
    if (!isConnected()) {
        ROS_WARN("串口未连接，无法发送数据");
        return false;
    }
    
    try {
        std::vector<uint8_t> payload;
        
        ROS_INFO("PWM值详细编码:");
        for (size_t i = 0; i < pwm_values.size(); i++) {
            int16_t pwm = std::max(static_cast<int16_t>(-32768), 
                                  std::min(static_cast<int16_t>(32767), pwm_values[i]));
            
            // 大端序打包
            uint16_t unsigned_pwm = static_cast<uint16_t>(pwm);
            payload.push_back((unsigned_pwm >> 8) & 0xFF);  // 高字节
            payload.push_back(unsigned_pwm & 0xFF);         // 低字节
            
            ROS_INFO("  PWM%zu: %d (0x%04X) -> [0x%02X, 0x%02X]", 
                     i+1, pwm, unsigned_pwm, payload[i*2], payload[i*2+1]);
        }
        
        std::vector<uint8_t> frame = encodeData(1, payload);
        
        // 打印完整数据包
        std::stringstream ss;
        for (uint8_t byte : frame) {
            ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        ROS_INFO("完整数据包: %s", ss.str().c_str());
        
        serial_port_->write(frame);
        ROS_INFO("发送推进器数据成功");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("发送数据失败: %s", e.what());
        return false;
    }
}

bool UnderwaterSerialCommunication::sendServoData(const std::vector<int16_t>& pwm_values) {
    if (!isConnected()) {
        ROS_WARN("串口未连接，无法发送数据");
        return false;
    }
    
    try {
        std::vector<uint8_t> payload;
        
        for (size_t i = 0; i < pwm_values.size(); i++) {
            int16_t pwm = std::max(static_cast<int16_t>(-32768), 
                                  std::min(static_cast<int16_t>(32767), pwm_values[i]));
            
            uint16_t unsigned_pwm = static_cast<uint16_t>(pwm);
            payload.push_back((unsigned_pwm >> 8) & 0xFF);
            payload.push_back(unsigned_pwm & 0xFF);
        }
        
        std::vector<uint8_t> frame = encodeData(3, payload);

        // 打印完整数据包
        std::stringstream ss;
        for (uint8_t byte : frame) {
            ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        ROS_INFO("完整数据包: %s", ss.str().c_str());

        serial_port_->write(frame);
        ROS_INFO("发送舵机数据成功");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("发送舵机数据失败: %s", e.what());
        return false;
    }
}

void UnderwaterSerialCommunication::thrusterCallback(const underwater_msgs::ThrustersPWM::ConstPtr& msg) {
    std::vector<int16_t> pwm_values = {msg->pwm1, msg->pwm2, msg->pwm3, msg->pwm4, msg->pwm5, msg->pwm6};
    
    std::stringstream ss;
    for (size_t i = 0; i < pwm_values.size(); i++) {
        ss << pwm_values[i];
        if (i < pwm_values.size() - 1) ss << ", ";
    }
    ROS_INFO("接收到的推进器PWM指令: [%s]", ss.str().c_str());
    
    sendThrusterData(pwm_values);
}

void UnderwaterSerialCommunication::servoCallback(const underwater_msgs::ServoPWM::ConstPtr& msg) {
    std::vector<int16_t> pwm_values = {msg->pwm_yaw, msg->pwm_pitch};

    std::stringstream ss;
    for (size_t i = 0; i < pwm_values.size(); i++) {
        ss << pwm_values[i];
        if (i < pwm_values.size() - 1) ss << ", ";
    }
    ROS_INFO("接收到的舵机PWM指令: [%s]", ss.str().c_str());
    sendServoData(pwm_values);
}

bool UnderwaterSerialCommunication::isConnected() const {
    return serial_port_ && serial_port_->isOpen();
}

void UnderwaterSerialCommunication::close() {
    should_stop_ = true;
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    if (serial_port_ && serial_port_->isOpen()) {
        serial_port_->close();
        ROS_INFO("串口已关闭");
    }
}

void UnderwaterSerialCommunication::receiveLoop() {
    while (!should_stop_ && ros::ok() && isConnected()) {
        try {
            if (serial_port_->available() > 0) {
                std::string data = serial_port_->read(1);
                if (!data.empty()) {
                    processByte(static_cast<uint8_t>(data[0]));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (const std::exception& e) {
            ROS_ERROR("接收数据时出错: %s", e.what());
            resetStateMachine();
        }
    }
}

void UnderwaterSerialCommunication::processByte(uint8_t byte) {
    switch (rx_state_) {
        case ReceiveState::WAIT_START_1:
            if (byte == START_BYTE_1) {
                rx_state_ = ReceiveState::WAIT_START_2;
                ROS_DEBUG("收到起始字节1");
            }
            break;
            
        case ReceiveState::WAIT_START_2:
            if (byte == START_BYTE_2) {
                rx_state_ = ReceiveState::WAIT_TYPE;
                ROS_DEBUG("收到起始字节2");
            } else {
                if (byte == START_BYTE_1) {
                    rx_state_ = ReceiveState::WAIT_START_2;
                } else {
                    rx_state_ = ReceiveState::WAIT_START_1;
                }
            }
            break;
            
        case ReceiveState::WAIT_TYPE:
            rx_data_type_ = byte;
            rx_state_ = ReceiveState::WAIT_LENGTH;
            ROS_DEBUG("收到数据类型: 0x%02X", byte);
            break;
            
        case ReceiveState::WAIT_LENGTH:
            rx_data_length_ = byte;
            rx_data_count_ = 0;
            rx_payload_.clear();
            if (rx_data_length_ == 0) {
                rx_state_ = ReceiveState::WAIT_CRC;
            } else {
                rx_state_ = ReceiveState::WAIT_DATA;
            }
            ROS_DEBUG("收到数据长度: %d", byte);
            break;
            
        case ReceiveState::WAIT_DATA:
            rx_payload_.push_back(byte);
            rx_data_count_++;
            if (rx_data_count_ >= rx_data_length_) {
                rx_state_ = ReceiveState::WAIT_CRC;
                ROS_DEBUG("数据负载接收完成: %zu字节", rx_payload_.size());
            }
            break;
            
        case ReceiveState::WAIT_CRC:
            rx_crc_ = byte;
            rx_state_ = ReceiveState::WAIT_END_1;
            ROS_DEBUG("收到CRC: 0x%02X", byte);
            break;
            
        case ReceiveState::WAIT_END_1:
            if (byte == END_BYTE_1) {
                rx_state_ = ReceiveState::WAIT_END_2;
                ROS_DEBUG("收到结束字节1");
            } else {
                ROS_WARN("结束字节1错误，重新开始");
                resetStateMachine();
                if (byte == START_BYTE_1) {
                    rx_state_ = ReceiveState::WAIT_START_2;
                }
            }
            break;
            
        case ReceiveState::WAIT_END_2:
            if (byte == END_BYTE_2) {
                ROS_DEBUG("收到结束字节2，数据包完整");
                handleCompletePacket();
            } else {
                ROS_WARN("结束字节2错误，重新开始");
                resetStateMachine();
                if (byte == START_BYTE_1) {
                    rx_state_ = ReceiveState::WAIT_START_2;
                }
            }
            resetStateMachine();
            break;
    }
}

void UnderwaterSerialCommunication::resetStateMachine() {
    rx_state_ = ReceiveState::WAIT_START_1;
    rx_data_type_ = 0;
    rx_data_length_ = 0;
    rx_payload_.clear();
    rx_crc_ = 0;
    rx_data_count_ = 0;
}

void UnderwaterSerialCommunication::handleCompletePacket() {
    try {
        ROS_INFO("接收到完整数据包: 类型=0x%02X, 长度=%d", rx_data_type_, rx_data_length_);
        
        if (rx_data_type_ == 2) {  // 传感器数据
            processSensorData(rx_data_length_, rx_payload_);
        } else {
            ROS_INFO("收到其他类型数据包: 0x%02X", rx_data_type_);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("处理数据包时出错: %s", e.what());
    }
}

void UnderwaterSerialCommunication::processSensorData(uint8_t length, const std::vector<uint8_t>& payload) {
    try {
        if (payload.size() >= 30) {  // 30字节传感器数据
            underwater_msgs::SensorData sensor_msg;
            
            // 解析传感器数据 (大端序)
            auto unpack_int16 = [&payload](size_t offset) -> int16_t {
                return static_cast<int16_t>((payload[offset] << 8) | payload[offset + 1]);
            };
            
            sensor_msg.accel_x = unpack_int16(0);
            sensor_msg.accel_y = unpack_int16(2);
            sensor_msg.accel_z = unpack_int16(4);
            sensor_msg.gyro_x = unpack_int16(6);
            sensor_msg.gyro_y = unpack_int16(8);
            sensor_msg.gyro_z = unpack_int16(10);
            sensor_msg.mag_x = unpack_int16(12);
            sensor_msg.mag_y = unpack_int16(14);
            sensor_msg.mag_z = unpack_int16(16);
            sensor_msg.angle_x = unpack_int16(18);
            sensor_msg.angle_y = unpack_int16(20);
            sensor_msg.angle_z = unpack_int16(22);
            sensor_msg.depth = unpack_int16(24);
            sensor_msg.temperature = unpack_int16(26);
            sensor_msg.humidity = unpack_int16(28);
            
            sensor_msg.header.stamp = ros::Time::now();
            sensor_pub_.publish(sensor_msg);
            
            ROS_INFO("传感器数据 - 加速度: (%d, %d, %d) 陀螺仪: (%d, %d, %d) 深度: %d, 温度: %d",
                     sensor_msg.accel_x, sensor_msg.accel_y, sensor_msg.accel_z,
                     sensor_msg.gyro_x, sensor_msg.gyro_y, sensor_msg.gyro_z,
                     sensor_msg.depth, sensor_msg.temperature);
        } else {
            ROS_WARN("传感器数据长度不足: %zu字节，期望至少30字节", payload.size());
        }
    } catch (const std::exception& e) {
        ROS_WARN("处理传感器数据失败: %s", e.what());
    }
}

} // namespace underwater_serial