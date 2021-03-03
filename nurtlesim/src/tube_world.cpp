/// \file
/// \brief
///
/// PARAMETERS:
///     wheel_base (double): distance between diff drive wheels
///     wheel_radius (double): wheel radius
///     left_wheel_joint (string): name of left wheel joint
///     right_wheel_joint (string): name of right wheel joint
///     x_variance (double): variance for x_dot noise
///     w_variance (double): variance for w noise
///     slip_min (double): minimum wheel slip
///     slip_max (double): maximum for wheel slip
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): Apollo wheel joint state values
///     tube_locations (visualization_msgs/MarkerArray): locations of tubes in tube_world
///     real_path (nav_msgs/Path): path of actual robot with noise
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): commanded body velocity
/// SERVICES:
///     

#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <random>

//global variables
static ros::Publisher pub;                      //joint state publisher
static ros::Publisher marker_pub;               //marker array publisher
static ros::Publisher path_pub;                 //path publisher
static sensor_msgs::JointState js;              //JointState message without slip
static sensor_msgs::JointState js_new;          //JointState message with slip
static sensor_msgs::JointState js_old;          //previous JointState message with slip
static rigid2d::DiffDrive dd;                   //differential drive object
static double base;                             //wheel separation
static double radius;                           //wheel radius
static const int frequency = 200;               //publishing frequency
static std::normal_distribution<> wheel_noise;  //simulate slip
static std::normal_distribution<> x_noise;      //simulate encoder noise
static std::normal_distribution<> w_noise;      //simulate encoder noise
static std::vector<double> tube_coordinates_x;  //x coordinates of tube locations
static std::vector<double> tube_coordinates_y;  //y coordinates of tube locations
static nav_msgs::Path path;                     //actual robot path
static std::string odom_frame_id;               //odom frame id


/// \brief random number generator
/// \returns reference to pseudo-random number generator object
std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

/// \brief track actual robot location
void odometry()
{
    rigid2d::Vector2D angs;

    //find change in wheel angles
    angs.x = js_new.position[0]-js_old.position[0];
    angs.y = js_new.position[1]-js_old.position[1];
    
    dd.updateConfiguration(angs);

    js_old = js_new;
}

/// \brief subscriber callback that converts twist message to wheel controls
/// \param twist - commanded body velocity
void velCallback(const geometry_msgs::TwistConstPtr &twist)
{
    if (twist->angular.z == 0 && twist->linear.x == 0)
    {
        return;
    }
    
    rigid2d::Twist2D Vb;
    rigid2d::Vector2D controls;

    //convert to Twist2D
    Vb.w = twist->angular.z+w_noise(get_random());
    Vb.x_dot = twist->linear.x+x_noise(get_random());
    Vb.y_dot = twist->linear.y;

    //calculate wheel controls
    controls = dd.calculateControls(Vb);

    //store controls in JointState message
    js.velocity[0] = controls.x;
    js.velocity[1] = controls.y;
    js.position[0] += controls.x/frequency;
    js.position[1] += controls.y/frequency;

    //add controls noise
    Vb.w += w_noise(get_random());
    Vb.x_dot += x_noise(get_random());

    //calculate wheel controls and add slip
    controls = dd.calculateControls(Vb);
    controls *= wheel_noise(get_random());

    //store noisy controls in JointState message
    js_new.velocity[0] = controls.x;
    js_new.velocity[1] = controls.y;
    js_new.position[0] += controls.x/frequency;
    js_new.position[1] += controls.y/frequency;

    //tack actual apollo configuration
    odometry();
}

/// \brief publishes JointState Message without noise
void publishJS()
{
    js.header.stamp = ros::Time::now();
    pub.publish(js);
}

/// \brief publishes marker array signifying tube locations
void setTubes()
{
    visualization_msgs::MarkerArray tubes;

    //cylinder orientation
    geometry_msgs::Quaternion rot;
    rot.x = 0;
    rot.y = 0;
    rot.z = 0;
    rot.w = 1;

    //marker color
    std_msgs::ColorRGBA col;
    col.r = 1;
    col.g = 1;
    col.b = 0;
    col.a = 1;

    //loop through all tubes an append marker messages to array
    for(int i = 0; i<tube_coordinates_x.size(); i++)
    {
        visualization_msgs::Marker tube;
        geometry_msgs::Point pos;
        

        //set shape and color
        tube.type = tube.CYLINDER;
        tube.color = col;

        //set pose
        pos.x = tube_coordinates_x[i];
        pos.y = tube_coordinates_y[i];
        pos.z = 0;
        tube.pose.position = pos;
        tube.pose.orientation = rot;

        //set scale
        tube.scale.x = 0.1;
        tube.scale.y = 0.1;
        tube.scale.z = 0.1;

        tube.ns = "real";
        tube.header.frame_id = odom_frame_id;
        tube.header.stamp = ros::Time::now();
        tube.id = i;

        tubes.markers.push_back(tube);
    }
    marker_pub.publish(tubes);
}

/// \brief publishes path messages of actual robot path
void pubPath()
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped ps;
    std_msgs::Header head;
    tf2::Quaternion rot;

    //header
    head.stamp = ros::Time::now();
    head.frame_id = odom_frame_id;
    ps.header = head;
    path.header = head;
    
    //position
    ps.pose.position.x = dd.getTransform().getX();
    ps.pose.position.y = dd.getTransform().getY();
    ps.pose.position.z = 0;

    //orientation
    rot.setRPY(0,0,dd.getTransform().getTheta());
    ps.pose.orientation = tf2::toMsg(rot);

    path.poses.push_back(ps);
    path_pub.publish(path);
}

/// \brief broadcast transform from world to turtle
void broadcast()
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::Quaternion g_rot;         //geometry messages quaternion
    tf2::Quaternion rot;                     //tf2 quaternion
    geometry_msgs::TransformStamped trans;   //transform stamped message
    
    //time
    trans.child_frame_id = "turtle";
    trans.header.frame_id = "world";
    trans.header.stamp = ros::Time::now();

    //position
    trans.transform.translation.x = dd.getTransform().getX();
    trans.transform.translation.y = dd.getTransform().getY();
    trans.transform.translation.z = 0;

    //orientation
    rot.setRPY(0,0,dd.getTransform().getTheta());
    g_rot = tf2::toMsg(rot);
    trans.transform.rotation = g_rot;

    //broadcast
    br.sendTransform(trans);
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
    const ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, velCallback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("tube_locations", 10, true);
    path_pub = nh.advertise<nav_msgs::Path>("real_path", 10);

    //get parameters
    double x_var;
    double w_var;
    double slip_min;
    double slip_max;
    double wheel_var;
    double tube_radius;
    std::string left_wheel_joint;    //name of left wheel joint
    std::string right_wheel_joint;   //name of right wheel joint
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);
    ros::param::get("/x_variance", x_var);
    ros::param::get("/w_variance", w_var);
    ros::param::get("/slip_min", slip_min);
    ros::param::get("/slip_max", slip_max);
    ros::param::get("/tube_radius", tube_radius);
    ros::param::get("/tube_coordinates_x", tube_coordinates_x);
    ros::param::get("/tube_coordinates_y", tube_coordinates_y);
    ros::param::get("/odom_frame_id", odom_frame_id);

    // initialize noise distributions
    wheel_var = (slip_min+slip_max)/2;
    std::normal_distribution<> d(wheel_var, wheel_var-slip_min);
    std::normal_distribution<> d1(0, x_var);
    std::normal_distribution<> d2(0, w_var);
    wheel_noise = d;
    x_noise = d1;
    w_noise = d2;

    // initialize differential drive class
    dd = rigid2d::DiffDrive(base,radius);

    // initialize joint state message
    js.name.push_back(left_wheel_joint);
    js.name.push_back(right_wheel_joint);
    js.position.push_back(0);
    js.position.push_back(0);
    js.velocity.push_back(0);
    js.velocity.push_back(0);
    js_old.name.push_back(left_wheel_joint);
    js_old.name.push_back(right_wheel_joint);
    js_old.position.push_back(0);
    js_old.position.push_back(0);
    js_old.velocity.push_back(0);
    js_old.velocity.push_back(0);
    js_new.name.push_back(left_wheel_joint);
    js_new.name.push_back(right_wheel_joint);
    js_new.position.push_back(0);
    js_new.position.push_back(0);
    js_new.velocity.push_back(0);
    js_new.velocity.push_back(0);

    //initialize path msg
    path.header.frame_id = odom_frame_id;

    //publish tube locations
    setTubes();

    //looping rate
    ros::Rate r(frequency);
    
    while(ros::ok())
    {
        publishJS();
        pubPath();
        broadcast();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}