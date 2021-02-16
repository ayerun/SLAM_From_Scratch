/// \file
/// \brief 
///
/// PARAMETERS:
///     wheel_base (double): distance between diff drive wheels
///     wheel_radius (double): wheel radius
///     left_wheel_joint (string): name of left wheel joint
///     right_wheel_joint (string): name of right wheel joint
///     odom_frame_id (string): name of odom frame
///     body_frame_id (string): name of body frame
/// PUBLISHES:
///     odom (nav_msgs/Odometry): Apollo odometry
/// SUBSCRIBES:
///     /joint_states (sensor_msgs/JointState)
/// BROADCASTS:
///     transfrom from /odom to /base_footprint
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
#include <rigid2d/set_pose.h>

//global variables
static ros::Subscriber js_sub;                  //joint state subscriber
static ros::Publisher pub;                      //odometry publisher
static std::string odom_frame_id;               //odometry frame name
static std::string body_frame_id;               //body frame name
static std::string left_wheel_joint;            //left wheel joint name
static std::string right_wheel_joint;           //right wheel joint name
static sensor_msgs::JointState js_new;          //incoming joint state message
static sensor_msgs::JointState js_old;          //previous joint state message
static nav_msgs::Odometry odom;                 //odometry messgae
static geometry_msgs::Quaternion g_rot;         //geometry messages quaternion
static geometry_msgs::TransformStamped trans;   //transform stamped message
static tf2::Quaternion rot;                     //tf2 quaternion
static rigid2d::Vector2D angs;                  //wheel angles
static rigid2d::Vector2D controls;              //wheel velocities
static rigid2d::Twist2D Vb;                     //body velocity
static rigid2d::DiffDrive dd;                   //differential drive object
static double base;                             //wheel separation
static double radius;                           //wheel radius
static const int frequency = 200;               //publishing frequency

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
    //time
    odom.header.stamp = ros::Time::now();

    //twist
    odom.twist.twist.angular.z = Vb.w;
    odom.twist.twist.linear.x = Vb.x_dot;
    odom.twist.twist.linear.y = Vb.y_dot;

    //position
    odom.pose.pose.position.x = dd.getTransform().getX();
    odom.pose.pose.position.y = dd.getTransform().getY();

    //orientation
    rot.setRPY(0,0,dd.getTransform().getTheta());
    g_rot = tf2::toMsg(rot);
    odom.pose.pose.orientation = g_rot;

    //publish
    pub.publish(odom);
}

/// \brief broadcast transform from odom to base_footprint
void broadcast()
{
    static tf2_ros::TransformBroadcaster br;
    
    //time
    trans.header.stamp = ros::Time::now();

    //position
    trans.transform.translation.x = dd.getTransform().getX();
    trans.transform.translation.y = dd.getTransform().getY();
    trans.transform.translation.z = 0;

    //orientation
    trans.transform.rotation = g_rot;

    //broadcast
    br.sendTransform(trans);
}

/// \brief resets Apollo's odometry to the given pose
/// \param req - service request
///              x: x coordinate
///              y: y coordinate
///              theta: rotation
/// \param resp - service response (empty)
/// \return true when callback complete
bool setposeCallback(rigid2d::set_pose::Request &req, rigid2d::set_pose::Response &res)
{
    rigid2d::Vector2D pos;
    rigid2d::Transform2D new_pose;

    //Create transformation for new position and orientation
    pos.x = req.x;
    pos.y = req.y;
    new_pose = rigid2d::Transform2D(pos,req.theta);

    //reinitialize differential drive object
    dd = rigid2d::DiffDrive(base,radius,new_pose);
    
    return true;
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

    //initialize joint state message at 0s
    js_old.name.push_back(left_wheel_joint);
    js_old.name.push_back(right_wheel_joint);
    js_old.position.push_back(0);
    js_old.position.push_back(0);
    js_old.velocity.push_back(0);
    js_old.velocity.push_back(0);

    //set odometry message frame ids
    odom.child_frame_id = body_frame_id;
    odom.header.frame_id = odom_frame_id;

    //set transformstamped message frame ids
    trans.header.frame_id = odom_frame_id;
    trans.child_frame_id = body_frame_id;

    //initialize service
    ros::ServiceServer service = nh.advertiseService("set_pose",setposeCallback);

    ros::Rate r(frequency); //looping rate

    while(ros::ok())
    {
        ros::spinOnce();
        publishOdom();
        broadcast();
        r.sleep();
    }

    return 0;
}