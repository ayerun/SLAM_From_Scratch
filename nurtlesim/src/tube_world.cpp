/// \file
/// \brief Node that converts body velocity to diff drive wheel controls. Publishes wheel controls to joint_states
///
/// PARAMETERS:
///     wheel_base (double): distance between diff drive wheels
///     wheel_radius (double): wheel radius
///     left_wheel_joint (string): name of left wheel joint
///     right_wheel_joint (string): name of right wheel joint
///     x_variance (double): variance for x_dot noise
///     w_variance (double): variance for w noise
///     wheel_variance (double): variance for wheel slip
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
#include <random>

//global variables
static ros::Publisher pub;              //publisher
static sensor_msgs::JointState js;      //JointState message
static rigid2d::DiffDrive dd;           //differential drive object
static double base;                     //wheel separation
static double radius;                   //wheel radius
static const int frequency = 200;       //publishing frequency
static std::normal_distribution<> wheel_noise;
static std::normal_distribution<> x_noise;
static std::normal_distribution<> w_noise;


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

/// \brief subscriber callback that converts twist message to wheel controls
/// \param twist - commanded body velocity
void velCallback(const geometry_msgs::TwistConstPtr &twist)
{
    rigid2d::Twist2D Vb;
    // geometry_msgs::Twist cmd;
    rigid2d::Vector2D controls;

    //get twist
    // cmd = *twist;

    //convert to Twist2D
    Vb.w = twist->angular.z+w_noise(get_random());
    Vb.x_dot = twist->linear.x+x_noise(get_random());
    Vb.y_dot = twist->linear.y;

    //calculate wheel controls
    controls = dd.calculateControls(Vb);

    //add noise to wheel controls
    controls*=wheel_noise(get_random());

    //store controls in JointState message and add noise
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
    const ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, velCallback);

    //get parameters
    double x_var;
    double w_var;
    double slip_min;
    double slip_max;
    double wheel_var;
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

    ros::Rate r(frequency);           //looping rate
    
    while(ros::ok())
    {
        publishJS();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}