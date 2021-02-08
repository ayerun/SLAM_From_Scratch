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
static std::string left_wheel_joint;
static std::string right_wheel_joint;
static double base;
static double radius;

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

    //get parameters
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);

    return 0;
}