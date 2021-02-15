#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

static ros::Subscriber cmd_sub;
static ros::Publisher controls_pub;
static ros::Subscriber sensor_sub;
static ros::Publisher joint_pub;
static nuturtlebot::WheelCommands controls_msg;
static rigid2d::DiffDrive dd;
static double base;
static double radius;

void cmdCallback(const geometry_msgs::TwistConstPtr &twis)
{
    rigid2d::Twist2D Vb;
    rigid2d::Vector2D controls;

    Vb.x_dot = twis->linear.x;
    Vb.w = twis->angular.z;
    controls = dd.calculateControls(Vb);

    controls_msg.left_velocity = controls.x;
    controls_msg.right_velocity = controls.y;

    // controls_pub.publish(controls_msg);
}

void sensorCallback(const nuturtlebot::SensorDataConstPtr &data)
{
    return;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"apollo_interface");
    ros::NodeHandle nh;

    cmd_sub = nh.subscribe("/cmd_vel", 10, cmdCallback);
    sensor_sub = nh.subscribe("/sensor_data", 100, sensorCallback);
    controls_pub = nh.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 10);
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);


    //get parameters
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);

    //initialize differential drive object
    dd = rigid2d::DiffDrive(base,radius);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}