/// \file
/// \brief 
///
/// PARAMETERS:
///
/// PUBLISHES:
///     
/// SUBSCRIBES:
///     
/// SERVICES:
///     

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

//global variables
static ros::Publisher pub;
static ros::Subscriber vel_sub;

void velCallback(const geometry_msgs::TwistConstPtr &cmd)
{
    return;
}

int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;

    //initialize publishers and subscribers
    pub = nh.advertise<sensor_msgs::JointState>("joint_state", 10);
    vel_sub = nh.subscribe("cmd_vel", 10, velCallback);

    return 0;
}