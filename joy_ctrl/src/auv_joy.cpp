#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

class Turtle {
    ros::NodeHandle n;
    ros::NodeHandle private_nh{"~"};
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Timer timer;
    std_msgs::Float64MultiArray current_pwm;
public:
    //线速度 单位: m/s; 角速度 单位: rad/s
    double linear_speed_limit=1.0, angular_speed_limit=2.0;
    string robot_simulation;
    string auv_joy_ctl_topic;
    void callback(const sensor_msgs::Joy::ConstPtr &Joy);
    void timerCallback(const ros::TimerEvent&);
    Turtle() {
        private_nh.getParam("robot", robot_simulation);
        ROS_INFO("Loaded private parameter robot: %s", robot_simulation.c_str());
        //n.getParam("robot", robot_simulation);
        auv_joy_ctl_topic = "/" + robot_simulation + "/setpoint/pwm";
 
        cout << "robot_simulation: " << robot_simulation << "  auv_joy_ctl_topic: " << auv_joy_ctl_topic << endl;
        current_pwm.data.resize(6);
        for (int i = 0; i < 6; i++) {
            current_pwm.data[i] = 0.0;
        }
        pub = n.advertise<std_msgs::Float64MultiArray>(auv_joy_ctl_topic, 1);
        sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &Turtle::callback, this);
    
        timer = n.createTimer(ros::Duration(0.05), &Turtle::timerCallback, this);
    }
};

void Turtle::timerCallback(const ros::TimerEvent &event) {
    pub.publish(current_pwm);
}

void Turtle::callback(const sensor_msgs::Joy::ConstPtr &Joy) {
//    cout << "axes("<<Joy->axes.size()<<"): [";
//    for (int i = 0; i < Joy->axes.size(); i++) {
//        cout << Joy->axes.at(i) << " ,";
//    }
//    cout << "]" << endl << "buttons("<<Joy->buttons.size()<<"): [";
//    for (int i = 0; i < Joy->buttons.size(); i++) {
//        cout << Joy->buttons.at(i) << " ,";
//    }
//    cout << "]" << endl;
    if (Joy->buttons[6] != 0) {
        current_pwm.data[4] = -1;
        current_pwm.data[5] = 1;
    }
    else if (Joy->buttons[8] != 0) {
        current_pwm.data[4] = 5;
        current_pwm.data[5] = -5;
    }
    else{
        current_pwm.data[4] = 0;
        current_pwm.data[5] = 0;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle");
    Turtle turtle;
    ros::spin();
    return 0;
}