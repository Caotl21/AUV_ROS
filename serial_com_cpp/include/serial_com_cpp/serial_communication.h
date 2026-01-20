#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

namespace serial_com_cpp {

enum class DataMode {
    DECIMAL,
    HEXADECIMAL
};

class SerialCommunication {
private:
    ros::NodeHandle nh_;                    // ROS节点句柄
    ros::NodeHandle pnh_;                   // 私有节点句柄
    ros::Subscriber sub_;                   // 订阅者
    ros::Publisher pub_;                    // 发布者
    
    std::unique_ptr<serial::Serial> serial_port_;  // 串口对象
    
    // 参数
    std::string port_;  // 串口设备路径
    int baudrate_;
    int timeout_ms_;
    DataMode mode_;
    
    // 私有方法
    bool initializeSerial();
    std::vector<uint8_t> stringToBytes(const std::string& data);
    std::string bytesToHexString(const std::vector<uint8_t>& bytes);
    int bytesToDecimal(const std::vector<uint8_t>& bytes);

    std::mutex data_mutex;   // 互斥锁
    ros::Timer timer;
    
public:
    SerialCommunication();
    ~SerialCommunication();
    
    bool isConnected() const;
    bool sendData(const std::string& data);
    std::pair<bool, std::string> receiveData();
    
    // 回调函数
    void dataCallback(const std_msgs::String::ConstPtr& msg);
    
    // 工具函数
    static bool isValidDecimal(const std::string& str);
    static bool isValidHexadecimal(const std::string& str);
};

} // namespace serial_com_cpp

#endif // SERIAL_COMMUNICATION_H
