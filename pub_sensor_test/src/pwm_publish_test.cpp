/**
 * @file publisher_node.cpp
 * @brief ROS消息发布者示例节点
 * @brief 发布std_msgs::String类型的消息到"chatter"话题
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <underwater_msgs/ThrustersPWM.h>
#include <underwater_msgs/ServoPWM.h>
#include <underwater_msgs/LightPWM.h>
#include <sstream>
 
 int main(int argc, char **argv)
 {
    ros::init(argc, argv, "PWM_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher thruster_pub = nh.advertise<sensor_msgs::JointState>("thruster_states", 1000);
    ros::Publisher servo_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
    // ros::Publisher thruster_pub = nh.advertise<underwater_msgs::ThrustersPWM>("thrusters_pwm", 1000);
    // ros::Publisher servo_pub = nh.advertise<underwater_msgs::ServoPWM>("servo_pwm", 1000);
    // ros::Publisher light_pub = nh.advertise<underwater_msgs::LightPWM>("light_pwm", 1000);
    
    ros::Rate loop_rate(50);
    
    sensor_msgs::JointState thrusters_msg;
    thrusters_msg.name.push_back("thruster_1_joint");
    thrusters_msg.name.push_back("thruster_2_joint");
    thrusters_msg.name.push_back("thruster_3_joint");
    thrusters_msg.name.push_back("thruster_4_joint");
    thrusters_msg.name.push_back("thruster_5_joint");
    thrusters_msg.name.push_back("thruster_6_joint");
    thrusters_msg.effort.push_back(0);
    thrusters_msg.effort.push_back(0);
    thrusters_msg.effort.push_back(0);
    thrusters_msg.effort.push_back(0);
    thrusters_msg.effort.push_back(0);
    thrusters_msg.effort.push_back(0);
    sensor_msgs::JointState servo_msg;
    // servo_msg.name.push_back("servo_1_joint");
    // servo_msg.name.push_back("servo_2_joint");
    // servo_msg.effort.push_back(0);
    // servo_msg.effort.push_back(0);
    // underwater_msgs::ServoPWM servo_msg;
    // underwater_msgs::LightPWM light_msg;
    // 消息计数器
    int count = 0;
    int thruster_delta = 0;


    while (ros::ok())
    {
        if(count > 100)
        {
        switch(thruster_delta)
        {
            case -300:
                thruster_delta = 0;
                break;
            case 0:
                thruster_delta = 300;
                break;
            case 300:
                thruster_delta = -300;
                break;
        }
            count = 0;
        }

        thrusters_msg.header.stamp = ros::Time::now();
        thrusters_msg.effort[0]=1500 + thruster_delta;
        thrusters_msg.effort[1]=1500 + thruster_delta;
        thrusters_msg.effort[2]=1500 + thruster_delta;
        thrusters_msg.effort[3]=1500 + thruster_delta;
        thrusters_msg.effort[4]=1500 + thruster_delta;
        thrusters_msg.effort[5]=1500 + thruster_delta;
        thruster_pub.publish(thrusters_msg);

        servo_msg.header.stamp = ros::Time::now();
        // servo_msg.effort[0] = 1500 + thruster_delta;
        // servo_msg.effort[1] = 1500 + thruster_delta;
        servo_pub.publish(servo_msg);
        // servo_msg.header.stamp = ros::Time::now();
        // servo_msg.pwm_yaw = 1500 + thruster_delta;
        // servo_msg.pwm_pitch = 1500 + thruster_delta;
        // servo_pub.publish(servo_msg);
        
        // light_msg.header.stamp = ros::Time::now();
        // light_msg.pwm1 = 3500 + 10*thruster_delta;
        // light_msg.pwm2 = 3500 - 10*thruster_delta;
        // light_pub.publish(light_msg);
        
        /**
         * ros::spinOnce() 处理ROS回调函数
         * 对于简单的发布者，这允许ROS处理后台任务
         */
        ros::spinOnce();
        
        /**
         * 按照设定的频率休眠
         * 保持1Hz的发布频率
         */
        loop_rate.sleep();
        
        // 计数器递增
        ++count;
    }
    
    ROS_INFO("Publisher node shutting down.");
    
    return 0;
 }