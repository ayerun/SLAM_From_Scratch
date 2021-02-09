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
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

//global variables
static ros::Subscriber js_sub;              //joint state subscriber
static ros::Publisher pub;                  //odometry publisher
static tf2_ros::TransformBroadcaster br;    //transform broadcaster
static std::string odom_frame_id;           //odometry frame name
static std::string body_frame_id;           //body frame name
static std::string left_wheel_joint;        //left wheel joint name
static std::string right_wheel_joint;       //right wheel joint name
static sensor_msgs::JointState js_new;      //incoming joint state message
static sensor_msgs::JointState js_old;      //previous joint state message
static nav_msgs::Odometry odom;             //odometry messgae
static geometry_msgs::Quaternion g_rot;     //geometry messages quaternion
static tf2::Quaternion rot;                 //tf2 quaternion
static rigid2d::Vector2D angs;              //wheel angles
static rigid2d::Vector2D controls;          //wheel velocities
static rigid2d::Twist2D Vb;                 //body velocity
static rigid2d::DiffDrive dd;               //differential drive object
static double base;                         //wheel separation
static double radius;                       //wheel radius
static const int frequency = 200;           //publishing frequency

/// \brief subscriber callback that tracks odometry
/// \param js_msg - current joint state
void jsCallback(const sensor_msgs::JointStateConstPtr &js_msg)
{
    js_new = *js_msg;

    //find change in wheel angles
    angs.x = js_new.position[0]-js_old.position[0];
    angs.y = js_new.position[1]-js_old.position[1];

    //calculate twist from controls
    controls.x = js_new.velocity[0];
    controls.y = js_new.velocity[1];
    Vb = dd.calculateTwist(controls);

    //update robot configuration
    dd.updateConfiguration(angs);

    js_old = js_new;
}

/// \brief oublish odometry
void publishOdom()
{
    odom.header.stamp = ros::Time::now();
    odom.twist.twist.angular.z = Vb.w;
    odom.twist.twist.linear.x = Vb.x_dot;
    odom.twist.twist.linear.y = Vb.y_dot;
    odom.pose.pose.position.x = dd.getTransform().getX();
    odom.pose.pose.position.y = dd.getTransform().getY();
    rot.setRPY(0,0,dd.getTransform().getTheta());
    g_rot = tf2::toMsg(rot);
    odom.pose.pose.orientation = g_rot;
    pub.publish(odom);
}

/// \brief initializes node, subscriber, publisher, parameters, and objects
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function
int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;

    //initialize subscribers and publishers
    js_sub = nh.subscribe("joint_states", 10, jsCallback);
    pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    //get parameters
    ros::param::get("/odom_frame_id", odom_frame_id);
    ros::param::get("/body_frame_id", body_frame_id);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);

    //initialize differential drive object
    dd = rigid2d::DiffDrive(base,radius);

    js_old.name.push_back(left_wheel_joint);
    js_old.name.push_back(right_wheel_joint);
    js_old.position.push_back(0);
    js_old.position.push_back(0);
    js_old.velocity.push_back(0);
    js_old.velocity.push_back(0);

    odom.child_frame_id = body_frame_id;
    odom.header.frame_id = odom_frame_id;

    ros::Rate r(frequency);     //looping rate

    while(ros::ok())
    {
        ros::spinOnce();
        publishOdom();
        r.sleep();
    }

    return 0;
}