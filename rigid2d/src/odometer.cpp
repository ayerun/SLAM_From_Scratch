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
static std::string odom_frame_id;
static std::string body_frame_id;
static std::string left_wheel_joint;
static std::string right_wheel_joint;
static double base;
static double radius;

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

    //get parameters
    ros::param::get("/odom_frame_id", odom_frame_id);
    ros::param::get("/body_frame_id", body_frame_id);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);

    return 0;
}