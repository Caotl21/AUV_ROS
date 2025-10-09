#include "serial_com_cpp/serial_communication.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_com_node");
    
    try {
        serial_com_cpp::SerialCommunication serial_comm;
        
        if (!serial_comm.isConnected()) {
            ROS_ERROR("Failed to initialize serial communication");
            return -1;
        }
        
        ROS_INFO("Serial communication node started successfully");
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in serial communication node: %s", e.what());
        return -1;
    }
    
    return 0;
}
