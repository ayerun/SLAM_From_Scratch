/// \file
/// \brief Calculates and publishes odometry using joint states. Broadcasts transform from turtlebot to odom frame.
///
/// PARAMETERS:
///     wheel_base (double): distance between diff drive wheels
///     wheel_radius (double): wheel radius
///     left_wheel_joint (string): name of left wheel joint
///     right_wheel_joint (string): name of right wheel joint
///     odom_frame_id (string): name of odom frame
///     body_frame_id (string): name of body frame
///     tube_coordinates_x (double array): x coordinates of landmarks
///     tube_coordinates_y (double array): y coordinates of landmarks
/// PUBLISHES:
///     odom (nav_msgs/Odometry): robot odometry
///     odom path (nav_msgs/Path): path of robot according to wheel odometry
///     slam path (nav_msgs/Path): path of robot according to SLAM
/// SUBSCRIBES:
///     /joint_states (sensor_msgs/JointState): wheel joint state values
///     /fake_sensor (visualization_msgs/MarkerArray): fake sensor data
/// BROADCASTS:
///     transfrom from /odom to /base_footprint
///     transform from /map to /odom
/// SERVICES:
///     

#include <armadillo>
#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nuslam/nuslam.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rigid2d/set_pose.h>

//global variables
static ros::Subscriber js_sub;                  //joint state subscriber
static ros::Subscriber lidar_sub;
static ros::Publisher pub;                      //odometry publisher
static ros::Publisher path_odom_pub;            //path odometery publisher
static ros::Publisher path_slam_pub;            //path slam publisher
static std::string odom_frame_id;               //odometry frame name
static std::string body_frame_id;               //body frame name
static std::string left_wheel_joint;            //left wheel joint name
static std::string right_wheel_joint;           //right wheel joint name
static sensor_msgs::JointState js_new;          //incoming joint state message
static sensor_msgs::JointState js_old;          //previous joint state message
static nav_msgs::Odometry odom;                 //odometry messgae
static geometry_msgs::TransformStamped trans;   //transform stamped message
static rigid2d::Vector2D angs;                  //wheel angles
static rigid2d::Vector2D controls;              //wheel velocities
static rigid2d::Twist2D Vb;                     //body velocity
static rigid2d::DiffDrive dd;                   //differential drive object
static nuslam::ekf filter;                      //kalman filter
static double base;                             //wheel separation
static double radius;                           //wheel radius
static const int frequency = 200;               //publishing frequency
static std::vector<double> tube_coordinates_x;  //x coordinates of tube locations
static std::vector<double> tube_coordinates_y;  //y coordinates of tube locations
static rigid2d::Transform2D Tmap_robot = rigid2d::Transform2D();

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
    tf2::Quaternion rot;

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
    odom.pose.pose.orientation = tf2::toMsg(rot);

    //publish
    pub.publish(odom);
}

/// \brief publishes path messages of actual robot path
void publishOdomPath()
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped ps;
    std_msgs::Header head;
    tf2::Quaternion rot;

    //header
    head.stamp = ros::Time::now();
    head.frame_id = "world";
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
    path_odom_pub.publish(path);
}

// \brief publishes path messages of slam robot path
void publishSlamPath()
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped ps;
    std_msgs::Header head;
    tf2::Quaternion rot;

    //header
    head.stamp = ros::Time::now();
    head.frame_id = "world";
    ps.header = head;
    path.header = head;
    
    //position
    ps.pose.position.x = Tmap_robot.getX();
    ps.pose.position.y = Tmap_robot.getY();
    ps.pose.position.z = 0;

    //orientation
    rot.setRPY(0,0,Tmap_robot.getTheta());
    ps.pose.orientation = tf2::toMsg(rot);

    path.poses.push_back(ps);
    path_slam_pub.publish(path);
}

/// \brief broadcast transform from odom to base_footprint
void broadcast()
{
    static tf2_ros::TransformBroadcaster br;

    tf2::Quaternion rot;
    
    trans.header.frame_id = odom_frame_id;
    trans.child_frame_id = body_frame_id;

    //time
    trans.header.stamp = ros::Time::now();

    //position
    trans.transform.translation.x = dd.getTransform().getX();
    trans.transform.translation.y = dd.getTransform().getY();
    trans.transform.translation.z = 0;

    //orientation
    rot.setRPY(0,0,dd.getTransform().getTheta());
    trans.transform.rotation = tf2::toMsg(rot);

    //broadcast
    br.sendTransform(trans);
}

/// \brief broadcast transform from map to odom
void broadcast_map2odom()
{
    //calculate transform
    rigid2d::Transform2D Todom_robot = rigid2d::Transform2D(rigid2d::Vector2D(dd.getTransform().getX(),dd.getTransform().getY()),0);
    rigid2d::Transform2D Tmap_odom = Tmap_robot*Todom_robot.inv();

    //initialize
    static tf2_ros::TransformBroadcaster br2;
    tf2::Quaternion rot;
    geometry_msgs::TransformStamped trans2;

    //frames
    trans2.header.frame_id = "map";
    trans2.child_frame_id = odom_frame_id;
    
    //time
    trans2.header.stamp = ros::Time::now();

    //position
    trans2.transform.translation.x = Tmap_odom.getX();
    trans2.transform.translation.y = Tmap_odom.getY();
    trans2.transform.translation.z = 0;

    //orientation
    rot.setRPY(0,0,Tmap_odom.getTheta());
    trans2.transform.rotation = tf2::toMsg(rot);

    //broadcast
    br2.sendTransform(trans2);
}

/// \brief implements slam using fake data with known data association
/// \param data - fake data for slam
void fakeSensorCallback(const visualization_msgs::MarkerArrayPtr &data)
{
    static std::unordered_map<int,int> hash;    //landmark initialization map
    
    //predict
    filter.predict(Vb,dd.getTransform());

    //update loop
    int len = data->markers.size();
    for(int i = 0; i < len; i++)
    {
        visualization_msgs::Marker measurement = data->markers[i];

        // convert measurement to polar
        rigid2d::Vector2D location = rigid2d::Vector2D(measurement.pose.position.x,measurement.pose.position.y);
        arma::mat z = nuslam::toPolar(location);

        // get id
        int j = measurement.id+1;

        //initialize landmark
        if (hash.find(j) == hash.end())
        {
            hash[j] = 1;

            //calculate landmark location in map coordinates
            rigid2d::Vector2D robot_pos(filter.getState()(1,0),filter.getState()(2,0));
            rigid2d::Transform2D Tmr(robot_pos,filter.getState()(0,0));
            rigid2d::Vector2D landmark_loc = Tmr(location);

            filter.initialize_landmark(j,landmark_loc);
            std::cout << filter.getState() << std::endl;
        }

        //update
        filter.update(z,j);
    }

    // find transform from map to robot
    rigid2d::Vector2D pos = rigid2d::Vector2D(filter.getState()(1,0),filter.getState()(2,0));
    Tmap_robot = rigid2d::Transform2D(pos,0);
}

/// \brief implements slam using fake data with unknown data association
/// \param data - fake data for slam
void sensorCallback(const visualization_msgs::MarkerArrayPtr &data)
{
    static std::unordered_map<int,int> hash;    //landmark initialization map
    
    //predict
    filter.predict(Vb,dd.getTransform());

    //update loop
    for(int i = 0; i < data->markers.size(); i++)
    {
        visualization_msgs::Marker measurement = data->markers[i];

        // convert measurement to polar
        rigid2d::Vector2D location = rigid2d::Vector2D(measurement.pose.position.x,measurement.pose.position.y);
        arma::mat z = nuslam::toPolar(location);

        int j = filter.associateData(z)+1;

        if (j<0) {
            continue;
        }
        else if (hash.find(j) == hash.end()) {
            hash[j] = 1;

            //calculate landmark location in map coordinates
            rigid2d::Vector2D robot_pos(filter.getState()(1,0),filter.getState()(2,0));
            rigid2d::Transform2D Tmr(robot_pos,filter.getState()(0,0));
            rigid2d::Vector2D landmark_loc = Tmr(location);

            filter.initialize_landmark(j,landmark_loc);
        }

        filter.update(z,j);

    }

    // find transform from map to robot
    rigid2d::Vector2D pos = rigid2d::Vector2D(filter.getState()(1,0),filter.getState()(2,0));
    Tmap_robot = rigid2d::Transform2D(pos,0);
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

    bool data_association;

    //get parameters
    ros::param::get("/odom_frame_id", odom_frame_id);
    ros::param::get("/body_frame_id", body_frame_id);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);
    ros::param::get("/tube_coordinates_x", tube_coordinates_x);
    ros::param::get("/tube_coordinates_y", tube_coordinates_y);
    ros::param::get("/data_association",data_association);

    //initialize subscribers and publishers
    js_sub = nh.subscribe("joint_states", 10, jsCallback);
    pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    path_odom_pub = nh.advertise<nav_msgs::Path>("odom_path", 10);
    path_slam_pub = nh.advertise<nav_msgs::Path>("slam_path", 10);
    const ros::Subscriber fake_sensor_sub = nh.subscribe("fake_sensor", 10, fakeSensorCallback);


    filter = nuslam::ekf(tube_coordinates_x.size());

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

    ros::Rate r(frequency); //looping rate

    while(ros::ok())
    {
        ros::spinOnce();
        publishOdom();
        publishOdomPath();
        publishSlamPath();
        broadcast();
        broadcast_map2odom();
        r.sleep();
    }

    return 0;
}