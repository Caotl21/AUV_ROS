#include "serial_com_cpp/underwater_serial_communication.h"
#include <ros/ros.h>
#include <locale.h>

int main(int argc, char** argv) {

    setlocale(LC_ALL, "en_US.UTF-8");
    setlocale(LC_CTYPE, "en_US.UTF-8");

    ros::init(argc, argv, "underwater_serial_node");
    
    try {
        underwater_serial::UnderwaterSerialCommunication underwater_comm;
        
        if (!underwater_comm.isConnected()) {
            ROS_ERROR("Failed to initialize underwater serial communication");
            return -1;
        }
        
        ROS_INFO("水下机器人串口通信节点运行中...");
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in underwater serial communication node: %s", e.what());
        return -1;
    }
    
    return 0;
}