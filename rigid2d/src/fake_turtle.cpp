/// \file
/// \brief Node that converts body velocity to diff drive wheel controls. Publishes wheel controls to joint_states
///
/// PARAMETERS:
///     wheel_base (double): distance between diff drive wheels
///     wheel_radius (double): wheel radius
///     left_wheel_joint (string): name of left wheel joint
///     right_wheel_joint (string): name of right wheel joint
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): Apollo wheel joint state values
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): commanded body velocity
/// SERVICES:
///     

#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

//global variables
static ros::Publisher pub;              //publisher
static ros::Subscriber vel_sub;         //subscriber
static std::string left_wheel_joint;    //name of left wheel joint
static std::string right_wheel_joint;   //name of right wheel joint
static sensor_msgs::JointState js;      //JointState message
static geometry_msgs::Twist cmd;        //Twist message
static rigid2d::DiffDrive dd;           //differential drive object
static rigid2d::Twist2D Vb;             //Twist intermediate
static rigid2d::Vector2D controls;      //wheel controls
static double base;                     //wheel separation
static double radius;                   //wheel radius
static const int frequency = 200;       //publishing frequency


/// \brief subscriber callback that converts twist message to wheel controls
/// \param twist - commanded body velocity
void velCallback(const geometry_msgs::TwistConstPtr &twist)
{
    //get twist
    cmd = *twist;

    //convert to Twist2D
    Vb.w = cmd.angular.z;
    Vb.x_dot = cmd.linear.x;
    Vb.y_dot = cmd.linear.y;

    //calculate wheel controls
    controls = dd.calculateControls(Vb);

    //store controls in JointState message
    js.velocity[0] = controls.x;
    js.velocity[1] = controls.y;
    js.position[0] += controls.x/frequency;
    js.position[1] += controls.y/frequency;

    //tack apollo configuration
    dd.updateConfiguration(controls);
}

/// \brief publishes JointState Message
void publishJS()
{
    js.header.stamp = ros::Time::now();
    pub.publish(js);
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

    //initialize publishers and subscribers
    pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    vel_sub = nh.subscribe("cmd_vel", 10, velCallback);

    //get parameters
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);

    dd = rigid2d::DiffDrive(base,radius);

    js.name.push_back(left_wheel_joint);
    js.name.push_back(right_wheel_joint);
    js.position.push_back(0);
    js.position.push_back(0);
    js.velocity.push_back(0);
    js.velocity.push_back(0);

    ros::Rate r(frequency);           //looping rate
    
    while(ros::ok())
    {
        publishJS();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}