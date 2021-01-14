#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

turtlesim::PoseConstPtr tpose;

void poseCallback(const turtlesim::PoseConstPtr& pose)
 {
   tpose = pose;
 }

int main(int argc, char** argv)
 {
   ros::init(argc, argv, "turtle_rect");
   ros::NodeHandle nh;
   ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
   ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

   ros::spin();
 }