#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Circle
{
    public:
        Circle():
            nh{},
            pub(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10))
        {}

    private:
        ros::Publisher pub;
        ros::NodeHandle nh;
        double speed = 1;
        double radius = 1;

};



int main(int argc, char* argv[])
{
    ros::init(argc,argv,"follow_circle");
    // ros::NodeHandle nh;

    // pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // while(ros::ok())
    // {
    //     ros::spinOnce;
    // }

    ros::spin();
    return 1;
}