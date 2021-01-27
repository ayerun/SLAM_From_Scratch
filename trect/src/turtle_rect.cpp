/// \file
/// \brief Node that commands turtles in turtlesim to draw rectangles 
///
/// PARAMETERS:
///     max_xdot (int): max x velocity
///     max_wdot (int): max angular velocity
///     frequency (int): publishing frequency
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs/Twist): turtle controller
/// SUBSCRIBES:
///     /turtle1/pose (turtlesim/Pose): turtle location and orientation
/// SERVICES:
///     start (start): inputs x, y, width, height and commands turtle to draw a rectangle at (x,y) with dimensions "width" and "height"
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <trect/start.h>

enum class State {MOVE, TURN, STOP, STOP2move, STOP2turn};
static State state = State::STOP;

//global variables
static turtlesim::PoseConstPtr tpose; //turtle pose
static geometry_msgs::Twist twist;    //turtle twist command
static double max_xdot;                  //max velocity
static double max_wdot;                  //max angular velocity
static int frequency;                 //loopint frequency
static int sides = 0;                 //current side of rectangle
static bool wh = true;                //track width or height
static float width;                   //width of rectangle
static float height;                  //height of rectangle
static ros::ServiceClient clearClient;       //turtlesim clear service
static ros::ServiceClient teleportClient;    //turtlesim teleport absolute service
static ros::ServiceClient penClient;         //turtlesim set pen service
static ros::Subscriber pose_sub;             //pose subscriber
static ros::Publisher pub;                   //publisher to /cmd_vel

/// \brief subscriber callback that updates turtle pose
/// \param pose - pose of turtle
void poseCallback(const turtlesim::PoseConstPtr &pose)
{
    tpose = pose;   //tpose is pointer to pose
    return;
}

/// \brief commands the turtle to move a certain distance
/// \param dist - distance to move
void move(float dist)
{
    twist.linear.x = max_xdot;
    ros::Rate r(frequency);
    double begin = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < begin+(dist/max_xdot))
    {
        pub.publish(twist);
        r.sleep();
    }
    state = State::STOP2turn;
    return;
}

/// \brief commands the turtle to rotate 90 degrees
void turn()
{
    twist.angular.z = max_wdot;
    ros::Rate r(frequency);
    double begin = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < begin+(3.14/2/max_wdot))
    {
        pub.publish(twist);
        r.sleep();
    }
    state = State::STOP2move;
    return;
}

/// \brief commands the turtle to stop moving and rotating
void stop()
{
    twist.angular.z = 0;
    twist.linear.x = 0;
    pub.publish(twist);
    return;
}

/// \brief start service callback, teleports turtle, clears background,
///        and commands turtle to draw rectangle
/// \param req - service request
///              x: x coordinate for teleportation
///              y: y coordinate for teleportation
///              width: width of rectangle
///              height: height of rectangle
/// \param resp - service response (empty)
/// \return true when callback complete
bool startCallback(trect::start::Request &req, trect::start::Response &res)
{
    //looping rate
    ros::Rate r(frequency);

    //track sides of rectangle
    sides = 0;

    //teleport to new position
    turtlesim::TeleportAbsolute::Request tele_req;
    turtlesim::TeleportAbsolute::Response tele_res;
    tele_req.x = req.x;
    tele_req.y = req.y;
    teleportClient.call(tele_req,tele_res);

    //clear background
    std_srvs::Empty emp;
    clearClient.call(emp);

    //set pen color to pink
    turtlesim::SetPen::Request pen_req;
    turtlesim::SetPen::Response pen_resp;
    pen_req.r = 164;
    pen_req.g = 50;
    pen_req.b = 168;
    pen_req.width = 3;
    penClient.call(pen_req,pen_resp);

    //set rectangle dimensions
    width = req.width;
    height = req.height;

    //draw rectangle
    state = State::MOVE;
    return true;
}

/// \brief initializes node, subscriber, publisher, services, and parameters
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function (should never reach end of function)
int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle nh;

    //initialize subscriber and publisher
    pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    //initialize services
    clearClient = nh.serviceClient<std_srvs::Empty>("clear");
    clearClient.waitForExistence();
    teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("TeleportAbsolute");
    teleportClient.waitForExistence();
    penClient = nh.serviceClient<turtlesim::SetPen>("SetPen");
    penClient.waitForExistence();

    //get parameters
    ros::param::get("/max_xdot", max_xdot);
    ros::param::get("/max_wdot", max_wdot);
    ros::param::get("/frequency", frequency);
    ROS_INFO_STREAM(max_xdot);
    ROS_INFO_STREAM(max_wdot);
    ROS_INFO_STREAM(frequency);


    //initialize start service
    ros::ServiceServer service = nh.advertiseService("start", startCallback);

    ros::Rate r(frequency);     //looping rate
    ros::Duration(2).sleep();   //give 2 seconds for everything to initialize

    while(ros::ok())
    {
        switch(state)
        {   
            case State::MOVE:
                if (sides > 3)
                {
                    state == State::STOP;
                }
                else if(wh)
                {
                    move(width);
                    wh = false;
                    sides++;
                }
                else
                {
                    move(height);
                    wh = true;
                    sides++;
                }
                break;
            case State::TURN:
                turn();
                break;
            case State::STOP2move:
                stop();
                state = State::MOVE;
                break;
            case State::STOP2turn:
                stop();
                state = State::TURN;
                break;
            case State::STOP:
                stop();
                break;
            default:
                throw std::logic_error("Invalid State");
        }
        ros::spinOnce();  //process all callbacks
        r.sleep();
    }

    return 0;
}