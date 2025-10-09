#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>

class KeyboardTestNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    
public:
    KeyboardTestNode() {
        pub_ = nh_.advertise<std_msgs::String>("servo_serial", 10);
        sub_ = nh_.subscribe("serial_received", 10, &KeyboardTestNode::responseCallback, this);
        
        ROS_INFO("Keyboard test node started. Type data to send (type 'quit' to exit):");
    }
    
    void responseCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Response: %s", msg->data.c_str());
    }
    
    void run() {
        std::string input;
        
        while (ros::ok()) {
            std::cout << "Enter data (decimal/hex): ";
            std::getline(std::cin, input);
            
            // 去除首尾空格
            input.erase(0, input.find_first_not_of(" \t\r\n"));
            input.erase(input.find_last_not_of(" \t\r\n") + 1);
            
            if (input == "quit" || input == "exit" || input == "q") {
                ROS_INFO("Exiting...");
                break;
            }
            
            if (!input.empty()) {
                std_msgs::String msg;
                msg.data = input;
                pub_.publish(msg);
                ROS_INFO("Published: %s", input.c_str());
                
                // 处理回调
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_test_node");
    
    try {
        KeyboardTestNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in keyboard test node: %s", e.what());
        return -1;
    }
    
    return 0;
}
