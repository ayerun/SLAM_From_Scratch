#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <rigid2d/diff_drive.hpp>
#include <rigid2d/rigid2d.hpp>


void cmdCallback(const nuturtlebot::WheelCommands msg)
{
    CHECK(msg.left_velocity == 256);
    CHECK(msg.right_velocity == 256);
}


TEST_CASE("wheel command pure translation","[pure_translation]")
 {
    //setup
    ros::NodeHandle nh;
    const ros::Subscriber sub = nh.subscribe("/wheel_cmd",10,cmdCallback);
    const ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10,true);

    //publish
    geometry_msgs::Twist twis;
    twis.linear.x = 0.22;
    pub.publish(twis);

    //spin
    ros::Rate r(100);
    for(int i = 0; ros::ok() && i!=200; i++)
    {
        ros::spinOnce();
        r.sleep();
    }

}


void cmdCallback2(const nuturtlebot::WheelCommands msg)
{
    CHECK(msg.left_velocity == -93);
    CHECK(msg.right_velocity == 93);
}


TEST_CASE("wheel command pure rotation","[pure_rotation]")
{
    //setup
    ros::NodeHandle nh;
    const ros::Subscriber sub = nh.subscribe("wheel_cmd",10,cmdCallback2);
    const ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10,true);

    //publish
    geometry_msgs::Twist twis;
    twis.angular.z = 1;
    pub.publish(twis);

    //spin
    ros::Rate r(100.0);
    for(int i = 0; ros::ok() && i!=200; i++)
    {
        ros::spinOnce();
        r.sleep();
    }
}



void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    CHECK(rigid2d::almost_equal(msg->position[0], 2*rigid2d::PI));
    CHECK(rigid2d::almost_equal(msg->position[1], 2*rigid2d::PI));
}


TEST_CASE("sensor data test","[encoder]") 
{
    //setup
    ros::NodeHandle nh;
    const ros::Subscriber sub = nh.subscribe("joint_states",10,sensorCallback);
    const ros::Publisher pub = nh.advertise<nuturtlebot::SensorData>("sensor_data",10,true);

    //publish
    nuturtlebot::SensorData mydata;
    mydata.left_encoder = 4096;
    mydata.right_encoder = 4096;
    pub.publish(mydata);

    //spin
    ros::Rate r(100);
    for(int i = 0; ros::ok() && i!=200; i++)
    {
        ros::spinOnce();
        r.sleep();
    }
}