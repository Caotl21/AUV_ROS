/**
 * @file test_pwm_node.cpp
 * @brief ROS消息发布者示例节点
 * @brief 发布std_msgs::String类型的消息到"chatter"话题
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <underwater_msgs/ThrustersPWM.h>
#include <underwater_msgs/ServoPWM.h>
#include <underwater_msgs/LightPWM.h>
#include <std_msgs/Int8.h>
#include <sstream>

class PWMPublisher
{
public:
    PWMPublisher()
    {
        ros::NodeHandle nh_;
        thruster_pub = nh_.advertise<sensor_msgs::JointState>("thruster_states", 1500);
        servo_pub = nh_.advertise<sensor_msgs::JointState>("servo_states", 1500);
        joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1500);
        
        emergency_sub_ = nh_.subscribe("emergency_states", 10, &PWMPublisher::emergencyCallback, this);
        
        // 初始化推进器消息结构
        thrusters_msg.name.push_back("thruster_1_joint");
        thrusters_msg.name.push_back("thruster_2_joint");
        thrusters_msg.name.push_back("thruster_3_joint");
        thrusters_msg.name.push_back("thruster_4_joint");
        thrusters_msg.name.push_back("thruster_5_joint");
        thrusters_msg.name.push_back("thruster_6_joint");
        // 预先调整effort大小
        thrusters_msg.effort.resize(6, 1500);

        // 初始化伺服消息结构
        servo_msg.name.push_back("servo_1_joint");
        servo_msg.name.push_back("servo_2_joint");
        servo_msg.effort.resize(2, 1500);
        
        timer_ = nh_.createTimer(ros::Duration(0.8), &PWMPublisher::timerCallback, this);
    }

    ~PWMPublisher()
    {
        thrusters_msg.header.stamp = ros::Time::now();
        for(int i=0;i<6;i++){
            thrusters_msg.effort[i]=1500;
        }
        thruster_pub.publish(thrusters_msg);
        ros::Duration(0.1).sleep();
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if(emergency_state == 0){
            double current_pwm = thrusters_msg.effort[1];
            if(current_pwm + thruster_delta >= 1700){
                thruster_delta = -20;
            } else if(current_pwm + thruster_delta <= 1300){
                thruster_delta = 20;
            }
            thrusters_msg.header.stamp = ros::Time::now();
            thrusters_msg.effort[0]=current_pwm + thruster_delta;
            thrusters_msg.effort[1]=current_pwm + thruster_delta;
            thrusters_msg.effort[2]=current_pwm + thruster_delta;
            thrusters_msg.effort[3]=current_pwm + thruster_delta;
            thrusters_msg.effort[4]=current_pwm + thruster_delta;
            thrusters_msg.effort[5]=current_pwm + thruster_delta;
            // 在这里更新并发布推进器消息
            thruster_pub.publish(thrusters_msg);
            //更新伺服信息
            current_pwm = servo_msg.effort[0];
            if(current_pwm + thruster_delta >= 1700){
                thruster_delta = -20;
            } else if(current_pwm + thruster_delta <= 1300){
                thruster_delta = 20;
            }
            servo_msg.header.stamp = ros::Time::now();
            servo_msg.effort[0]=current_pwm + thruster_delta;
            servo_msg.effort[1]=current_pwm + thruster_delta;
            // 在这里更新并发布推进器消息
            servo_pub.publish(servo_msg);
        }
        else{
            //发布紧急停止消息
            if(emergency_state == 1)
            {
                thrusters_msg.header.stamp = ros::Time::now();
                thrusters_msg.effort[0] = 0xFFFF;
            }
            //发布复位消息
            if(emergency_state == 2)
            {
                thrusters_msg.header.stamp = ros::Time::now();
                thrusters_msg.effort[0] = 0xEFFF;
                emergency_state = 0;
            }
            thruster_pub.publish(thrusters_msg);
        }
    }

    void emergencyCallback(const std_msgs::Int8::ConstPtr& msg)
    {
        emergency_state = msg->data;
    }

private:
    //发布者
    ros::Timer timer_;
    ros::Publisher thruster_pub;
    ros::Publisher servo_pub;
    ros::Publisher joint_pub;
    //订阅者
    ros::Subscriber emergency_sub_;
    //消息等其他变量
    sensor_msgs::JointState thrusters_msg;
    sensor_msgs::JointState servo_msg;
    int emergency_state=0;
    int thruster_delta=20;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_pwm_node");
    PWMPublisher pwm_publisher;
    ros::spin();
    return 0;
}