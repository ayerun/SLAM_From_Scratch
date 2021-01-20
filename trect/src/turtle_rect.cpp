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

//change to static
turtlesim::PoseConstPtr tpose;
geometry_msgs::Twist twist;
int max_xdot;
int max_wdot;
int frequency;
bool wh = true;
float x;
float y;
float width;
float height;
float w_sleep;
float h_sleep;
double t_sleep;


void poseCallback(const turtlesim::PoseConstPtr &pose)
{
    tpose = pose;
}

void move(ros::Publisher pub, float dist)
{
    twist.linear.x = max_xdot;
    pub.publish(twist);
    ros::Duration(dist/max_xdot).sleep();
    state = State::STOP2turn;
    return;
}

void turn(ros::Publisher pub)
{
    twist.angular.z = max_wdot;
    ros::Rate r(frequency);
    double begin = ros::Time::now().toSec();
    while (ros::Time::now().toSec() < begin+(3.14/2/max_wdot))
    {
        pub.publish(twist);
        r.sleep();
    }
    // ros::Duration(3.14/2/max_wdot).sleep();
    state = State::STOP2move;
    return;
}

void stop(ros::Publisher pub)
{
    twist.angular.z = 0;
    twist.linear.x = 0;
    pub.publish(twist);
    return;
}

float round2(float var) 
{ 
    //round to 2 decminal places
    float value = (int)(var * 100 + .5); 
    return (float)value / 100; 
} 


bool startCallback(trect::start::Request &req, trect::start::Response &res)
{
    //intialize services
    ros::NodeHandle nh;
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");
    clearClient.waitForExistence();
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("TeleportAbsolute");
    teleportClient.waitForExistence();

    ros::Rate r(frequency);

    //teleport to new position
    turtlesim::TeleportAbsolute::Request tele_req;
    turtlesim::TeleportAbsolute::Response tele_res;
    tele_req.x = req.x;
    tele_req.y = req.y;
    bool success = teleportClient.call(tele_req,tele_res);

    //clear background
    std_srvs::Empty emp;
    clearClient.call(emp);

    x = req.x;
    y = req.y;
    width = req.width;
    height = req.height;
    w_sleep = width/max_xdot;
    h_sleep = height/max_xdot;
    t_sleep = 0.1;
    // t_sleep = round2(3.142/max_wdot);

    //draw rectangle
    state = State::MOVE;
    return true;
}

int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle nh;

    //initialize subscriber and publisher
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    //get parameters
    ros::param::get("/max_xdot", max_xdot);
    ros::param::get("/max_wdot", max_wdot);
    ros::param::get("/frequency", frequency);
    ROS_INFO_STREAM(max_xdot);
    ROS_INFO_STREAM(max_wdot);
    ROS_INFO_STREAM(frequency);


    // //initialize services
    ros::ServiceClient teleportClient = nh.serviceClient<turtlesim::TeleportAbsolute>("TeleportAbsolute");
    teleportClient.waitForExistence();
    // ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>("SetPen");
    // penClient.waitForExistence();
    ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("clear");
    clearClient.waitForExistence();
    ros::ServiceServer service = nh.advertiseService("start", startCallback);

    ros::Rate r(frequency);
    ros::Duration(2).sleep();

    while(ros::ok())
    {
        switch(state)
        {   
            case State::MOVE:
                ROS_ERROR_STREAM("MOVIN");
                if(wh)
                {
                    move(pub,width);
                    wh = false;
                }
                else
                {
                    move(pub,height);
                    wh = true;
                }
                break;
            case State::TURN:
                ROS_ERROR_STREAM("TURNIN");
                turn(pub);
                break;
            case State::STOP2move:
                stop(pub);
                state = State::MOVE;
                break;
            case State::STOP2turn:
                stop(pub);
                state = State::TURN;
                break;
            case State::STOP:
                stop(pub);
                break;
            default:
                throw std::logic_error("Invalid State");
        }
        ros::spinOnce();  //process all callbacks
        r.sleep();
    }

    //test pen
  //  turtlesim::SetPen::Request pen_req;
  //  turtlesim::SetPen::Response pen_resp;
  //  pen_req.off = 1;
  //  bool success1 = penClient.call(pen_req,pen_resp);

    //test teleport
    // turtlesim::TeleportAbsolute::Request pos;
    // turtlesim::TeleportAbsolute::Response resp;
    // pos.x = 2;
    // pos.y = 3;
    // bool success2 = teleportClient.call(pos,resp);
    // turn(pub);
    // go(pub,1);


}