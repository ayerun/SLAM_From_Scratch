/// \file
/// \brief 
///
/// PARAMETERS:
///
/// PUBLISHES:
///     
/// SUBSCRIBES:
///     /joint_states (sensor_msgs/JointState)
/// SERVICES:
///     

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//global variables
static ros::Subscriber js_sub;
static ros::Publisher pub;

void jsCallback(const sensor_msgs::JointStateConstPtr &js)
{
    return;
}

int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;

    //initialize subscribers and publishers
    js_sub = nh.subscribe("joint_state", 10, jsCallback);
    pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    return 0;
}