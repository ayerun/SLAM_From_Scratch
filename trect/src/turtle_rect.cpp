#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <trect/start.h>

//change to static
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

void turn(ros::Publisher pub)
{
    geometry_msgs::Twist turt_twist;
    turt_twist.angular.z = max_wdot;

    ros::Rate r(frequency);

    double begin = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < begin+(3.14159/2/max_wdot))
    {
        pub.publish(turt_twist);
        r.sleep();
    }
    stop(pub);
}

void go(ros::Publisher pub, float dist)
{
    geometry_msgs::Twist turt_twist;
    turt_twist.linear.x = max_xdot;

    ros::Rate r(frequency);

    double begin = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < begin+(dist/max_xdot))
    {
        pub.publish(turt_twist);
        r.sleep();
    }
    stop(pub);
}

void poseCallback(const turtlesim::PoseConstPtr &pose)
{
    tpose = pose;
}

bool startCallback(trect::start::Request &req, trect::start::Response &res)
{
    //intialize services
    ros::NodeHandle nh;
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");
    clearClient.waitForExistence();
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("TeleportAbsolute");
    teleportClient.waitForExistence();

    //teleport to new position
    turtlesim::TeleportAbsolute::Request tele_req;
    turtlesim::TeleportAbsolute::Response tele_res;
    tele_req.x = req.x;
    tele_req.y = req.y;
    bool success = teleportClient.call(tele_req,tele_res);

    //clear background
    std_srvs::Empty emp;
    clearClient.call(emp);

    //draw rectangle
    // go(pub,width);
    
    return true;
}

int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle nh;

    //initialize subscriber and publisher
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    //get parameters
    ros::param::get("/max_xdot", max_xdot);
    ros::param::get("/max_wdot", max_wdot);
    ros::param::get("/frequency", frequency);
    ROS_INFO_STREAM(max_xdot);
    ROS_INFO_STREAM(max_wdot);
    ROS_INFO_STREAM(frequency);

    // //initialize services
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("TeleportAbsolute");
    teleportClient.waitForExistence();
    ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("SetPen");
    penClient.waitForExistence();
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");
    clearClient.waitForExistence();
    ros::ServiceServer service = nh.advertiseService("start", startCallback);

    ros::Rate r(frequency);
    ros::Duration(2).sleep();

    //test pen
  //  turtlesim::SetPen::Request pen_req;
  //  turtlesim::SetPen::Response pen_resp;
  //  pen_req.off = 1;
  //  bool success1 = penClient.call(pen_req,pen_resp);

    //test teleport
    turtlesim::TeleportAbsolute::Request pos;
    turtlesim::TeleportAbsolute::Response resp;
    pos.x = 2;
    pos.y = 3;
    bool success2 = teleportClient.call(pos,resp);
    turn(pub);
    go(pub,1);

    // while(ros::ok())
    // {
    //   ros::spinonce();  //process all callbacks
    //   r.sleep();
    // }

    ros::spin();
}