#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtle_robot/control.h>

static ros::Publisher pub;
static double speed;
static double rad;
static geometry_msgs::Twist cmd;
static int frequency = 200;


void follow_circle()
{
    pub.publish(cmd);
}

bool controlCallback(nuturtle_robot::control::Request &req, nuturtle_robot::control::Response &res)
{
    if(req.clockwise)
    {
        cmd.linear.x = -speed;
        cmd.angular.z = -speed/rad;
        res.final = "Moving clockwise";
    } 
    else
    {
        cmd.linear.x = speed;
        cmd.angular.z = speed/rad;
        res.final = "Moving counter clockwise";
    }
    if(req.stop)
    {
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        res.final = "Apollo Stopped";
    }

    return true;
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"follow_circle");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //get parameters
    ros::param::get("/follow_circle/speed", speed);
    ros::param::get("/follow_circle/radius", rad);

    //enforce speed is not velocity
    if(speed < 0)
    {
        speed = -speed;
    }

    cmd.linear.x = speed;
    cmd.angular.z = speed/rad;

    ros::ServiceServer service = nh.advertiseService("control",controlCallback);

    ros::Rate r(frequency);
    while(ros::ok())
    {
        follow_circle();
        ros::spinOnce();
        r.sleep();
    }

    return 1;
}