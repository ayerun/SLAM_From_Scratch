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
static sensor_msgs::JointState js;
static std::string left_wheel_joint;    //name of left wheel joint
static std::string right_wheel_joint;   //name of right wheel joint
static nuturtlebot::SensorData data_new;
static nuturtlebot::SensorData data_old;
static nuturtlebot::WheelCommands controls_msg;
static rigid2d::DiffDrive dd;
static double base;
static double radius;
static double mapping_constant;
static int frequency = 200;

void cmdCallback(const geometry_msgs::TwistConstPtr &twis)
{
    rigid2d::Twist2D Vb;
    rigid2d::Vector2D controls;

    Vb.x_dot = twis->linear.x;
    Vb.w = twis->angular.z;
    controls = dd.calculateControls(Vb);

    //mapping
    controls_msg.left_velocity = 256*controls.x/mapping_constant;
    controls_msg.right_velocity = 256*controls.y/mapping_constant;


    controls_pub.publish(controls_msg);
}

void sensorCallback(const nuturtlebot::SensorDataConstPtr &data)
{
    rigid2d::Vector2D angs;

    data_new = *data;
    
    //find change in encoder ticks
    angs.x = data_new.left_encoder - data_old.left_encoder;
    angs.y = data_new.right_encoder - data_old.right_encoder;

    //convert ticks to angles
    angs.x = angs.x/4096;
    angs.y = angs.y/4096;
    
    //populate message
    js.velocity[0] = angs.x;
    js.velocity[1] = angs.y;
    js.position[0] = data_new.left_encoder/4096;
    js.position[1] = data_new.right_encoder/4096;
    js.header.stamp = ros::Time::now();

    //publish joint state
    joint_pub.publish(js);

    return;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"apollo_interface");
    ros::NodeHandle nh;

    cmd_sub = nh.subscribe("/cmd_vel", 10, cmdCallback);
    sensor_sub = nh.subscribe("/sensor_data", 10, sensorCallback);
    controls_pub = nh.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 10);
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);


    //get parameters
    ros::param::get("/wheel_base", base);
    ros::param::get("/wheel_radius", radius);
    ros::param::get("/left_wheel_joint", left_wheel_joint);
    ros::param::get("/right_wheel_joint", right_wheel_joint);

    //initialize differential drive object
    dd = rigid2d::DiffDrive(base,radius);
    mapping_constant = 0.22/radius;

    js.name.push_back(left_wheel_joint);
    js.name.push_back(right_wheel_joint);
    js.position.push_back(0);
    js.position.push_back(0);
    js.velocity.push_back(0);
    js.velocity.push_back(0);

    ros::Rate r(frequency);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}