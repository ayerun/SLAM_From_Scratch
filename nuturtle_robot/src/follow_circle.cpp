#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

static ros::Publisher pub;
static double speed;
static double rad;
static geometry_msgs::Twist cmd;
static int frequency = 200;


void follow_circle()
{
    pub.publish(cmd);
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"follow_circle");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //get parameters
    ros::param::get("/follow_circle/speed", speed);
    ros::param::get("/follow_circle/radius", rad);

    ros::Rate r(frequency);

    cmd.linear.x = speed;
    cmd.angular.z = rad/speed;

    while(ros::ok())
    {
        follow_circle();
        ros::spinOnce;
        r.sleep();
    }

    return 1;
}