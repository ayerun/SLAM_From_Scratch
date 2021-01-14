#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

turtlesim::PoseConstPtr tpose;
int max_xdot;
int max_wdot;
int frequency;
int x = 2;
int y = 3;
int width = 4;
int height = 5;


void stop(ros::Publisher pub)
{
    geometry_msgs::Twist turt_twist;
    pub.publish(turt_twist);
}

void turn(ros::Publisher pub, ros::Rate r)
{
    geometry_msgs::Twist turt_twist;
    turt_twist.angular.z = 3.14159;
    double begin = ros::Time::now().toSec();
    
    while (ros::Time::now().toSec() < begin+1)
    {
        pub.publish(turt_twist);
        r.sleep();
    }
    stop(pub);
}

void poseCallback(const turtlesim::PoseConstPtr& pose)
 {
   tpose = pose;
 }

int main(int argc, char** argv)
 {
   ros::init(argc, argv, "turtle_rect");
   ros::NodeHandle nh;

   ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

   ros::param::get("/max_xdot", max_xdot);
   ros::param::get("/max_wdot", max_wdot);
   ros::param::get("/frequency", frequency);

   ROS_INFO_STREAM(max_xdot);
   ROS_INFO_STREAM(max_wdot);
   ROS_INFO_STREAM(frequency);

   ros::Rate r(frequency);

   ros::Duration(3).sleep();
   turn(pub,r);

//    while(ros::ok())
//    {
//        turn(pub,r);
//    }

   ros::spin();
 }