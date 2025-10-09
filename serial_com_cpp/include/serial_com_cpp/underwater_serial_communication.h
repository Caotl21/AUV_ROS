#ifndef UNDERWATER_SERIAL_COMMUNICATION_H
#define UNDERWATER_SERIAL_COMMUNICATION_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <underwater_msgs/ThrustersPWM.h>
#include <underwater_msgs/SensorData.h>
#include <underwater_msgs/ServoPWM.h>
#include <vector>
#include <thread>
#include <atomic>

namespace underwater_serial {

enum class ReceiveState {
    WAIT_START_1 = 0,   // 等待第一个起始字节 0xAA
    WAIT_START_2 = 1,   // 等待第二个起始字节 0xBB
    WAIT_TYPE = 2,      // 等待数据类型字节
    WAIT_LENGTH = 3,    // 等待数据长度字节
    WAIT_DATA = 4,      // 等待数据负载
    WAIT_CRC = 5,       // 等待CRC校验字节
    WAIT_END_1 = 6,     // 等待第一个结束字节 0xCC
    WAIT_END_2 = 7      // 等待第二个结束字节 0xDD
};

class UnderwaterSerialCommunication {
public:
    UnderwaterSerialCommunication();
    ~UnderwaterSerialCommunication();
    
    bool isConnected() const;
    void close();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber thruster_sub_;
    ros::Subscriber servo_sub_;
    ros::Publisher sensor_pub_;
    
    // 串口相关
    std::unique_ptr<serial::Serial> serial_port_;
    std::string port_;
    int baudrate_;
    double timeout_;
    
    // 协议定义
    static const uint8_t START_BYTE_1 = 0xAA;
    static const uint8_t START_BYTE_2 = 0xBB;
    static const uint8_t END_BYTE_1 = 0xCC;
    static const uint8_t END_BYTE_2 = 0xDD;
    
    // 状态机变量
    ReceiveState rx_state_;
    uint8_t rx_data_type_;
    uint8_t rx_data_length_;
    std::vector<uint8_t> rx_payload_;
    uint8_t rx_crc_;
    uint8_t rx_data_count_;
    
    // CRC8查找表
    std::vector<uint8_t> crc8_table_;
    
    // 接收线程
    std::thread receive_thread_;
    std::atomic<bool> should_stop_;
    
    // 初始化方法
    bool initializeSerial();
    void generateCRC8Table();
    uint8_t calculateCRC8(const std::vector<uint8_t>& data);
    
    // 数据编码解码
    std::vector<uint8_t> encodeData(uint8_t data_type, const std::vector<uint8_t>& payload);
    
    // 发送方法
    bool sendThrusterData(const std::vector<int16_t>& pwm_values);
    bool sendServoData(const std::vector<int16_t>& pwm_values);
    
    // 接收处理
    void receiveLoop();
    void processByte(uint8_t byte);
    void resetStateMachine();
    void handleCompletePacket();
    void processSensorData(uint8_t length, const std::vector<uint8_t>& payload);
    
    // ROS回调
    void thrusterCallback(const underwater_msgs::ThrustersPWM::ConstPtr& msg);
    void servoCallback(const underwater_msgs::ServoPWM::ConstPtr& msg);
};

} // namespace underwater_serial

#endif // UNDERWATER_SERIAL_COMMUNICATION_H