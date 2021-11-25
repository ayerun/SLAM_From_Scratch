/// \file
/// \brief simulates real robot by adding noise the velocity commands and slip to wheels. tracks configuration of real robot
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
///     tube_radius (double): radius of tubes
///     tube_coordinates_x (double array): x coordinates of landmarks
///     tube_coordinates_y (double array): y coordinates of landmarks
///     odom_frame_id (string): odom frame
///     tube_var (double): variance of fake sensor data
///     max_distance (double): max distance of sensor
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): Apollo wheel joint state values
///     tube_locations (visualization_msgs/MarkerArray): locations of tubes in tube_world
///     real_path (nav_msgs/Path): path of actual robot with noise
///     fake_sensor (visualization_msgs/MarkerArray): fake sensor readings of markers
///     fake_laser (sensor_msgs/LaserScan): simulated lidar data
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): commanded body velocity
/// SERVICES:
///     

#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nurtlesim/nurtlesim.hpp>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
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
static ros::Publisher pub;                          //joint state publisher
static ros::Publisher marker_pub;                   //marker array publisher
static ros::Publisher path_pub;                     //path publisher
static ros::Publisher fakeTube_pub;                 //fake sensor data
static ros::Publisher fakeLaser_pub;                //fake lidar data
static sensor_msgs::JointState js;                  //JointState message without slip
static sensor_msgs::JointState js_new;              //JointState message with slip
static sensor_msgs::JointState js_old;              //previous JointState message with slip
static rigid2d::DiffDrive dd;                       //differential drive object
static double base;                                 //wheel separation
static double radius;                               //wheel radius
static double max_distance;                         //max sensing distance for tube markers
static double tube_radius;                          //radius of tube markers
static double range_min;                            //min distance for scan
static double range_max;                            //max distance for scan
static double time_increment;                       //time between measurements
static double angle_increment;                      //angular distance between measurements
static double laser_var;                            //variance of laser scanner
static double resolution;                           //resolution of laser scanner
static double width;                                //boarder width
static double length;                               //boarder length
static int num_samples;                             //number of points per scan
static const double scan_time = 0.2;                //time between laser scans
static const int frequency = 200;                   //publishing frequency
static std::normal_distribution<> wheel_noise;      //simulate slip
static std::normal_distribution<> x_noise;          //simulate encoder noise
static std::normal_distribution<> w_noise;          //simulate encoder noise
static std::normal_distribution<> tube_noise;       //measurement noise
static std::normal_distribution<> scan_noise;       //laser scan noise
static std::vector<double> tube_coordinates_x;      //x coordinates of tube locations
static std::vector<double> tube_coordinates_y;      //y coordinates of tube locations
static nav_msgs::Path path;                         //actual robot path
static std::string odom_frame_id;                   //odom frame id
static std::string body_frame_id;                   //body frame id
static std::vector<rigid2d::Vector2D> landmarks_world;               //list of landmark locations


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
    //do nothing if robot is stopped
    if (twist->angular.z == 0 && twist->linear.x == 0)
    {
        return;
    }
    
    rigid2d::Twist2D Vb;
    rigid2d::Vector2D controls;

    //convert to Twist2D
    Vb.w = twist->angular.z;
    Vb.x_dot = twist->linear.x;
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
        tube.scale.x = tube_radius;
        tube.scale.y = tube_radius;
        tube.scale.z = 0.1;

        tube.ns = "real";
        tube.header.frame_id = "world";
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

/// \brief publishes fake sensor readings from the turtle frame
void fakeSensor()
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
    col.r = 0;
    col.g = 1;
    col.b = 0;
    col.a = 1;

    //loop through all tubes an append marker messages to array
    for(int i = 0; i<tube_coordinates_x.size(); i++)
    {
        visualization_msgs::Marker tube;
        geometry_msgs::Point pos;
        

        //set shape, color, and lifetime
        tube.type = tube.CYLINDER;
        tube.color = col;
        tube.lifetime = ros::Duration(0.1);

        //identification
        tube.ns = "fake";
        tube.header.frame_id = "turtle";
        tube.header.stamp = ros::Time::now();
        tube.id = i;

        //set pose
        rigid2d::Vector2D location = rigid2d::Vector2D(tube_coordinates_x[i],tube_coordinates_y[i]);
        rigid2d::Transform2D Tw_tube = rigid2d::Transform2D(location,0);
        rigid2d::Transform2D Tw_turtle = dd.getTransform();
        rigid2d::Transform2D Tturtle_tube = Tw_turtle.inv()*Tw_tube;
        pos.x = Tturtle_tube.getX()+tube_noise(get_random());
        pos.y = Tturtle_tube.getY()+tube_noise(get_random());
        pos.z = 0;
        tube.pose.position = pos;
        tube.pose.orientation = rot;

        //set scale
        tube.scale.x = tube_radius;
        tube.scale.y = tube_radius;
        tube.scale.z = 0.1;

        //enforce max distance
        double distance = sqrt(pow(Tturtle_tube.getX(),2)+pow(Tturtle_tube.getY(),2));
        if (distance < max_distance)
        {
            tubes.markers.push_back(tube);
        }
    }
    fakeTube_pub.publish(tubes);
}

/// \brief store point of intersection without reflection
/// \param ipoint - point of intersection
/// \param p1 - point on ray
/// \param p2 - point on ray
/// \param intersections - vector to store intersections
/// \returns vector of intersections
std::vector<rigid2d::Vector2D> storeIntersections(const rigid2d::Vector2D ipoint, const rigid2d::Vector2D p1, const rigid2d::Vector2D p2, std::vector<rigid2d::Vector2D> intersections)
{
    if(ipoint.x > p1.x && ipoint.x < p2.x && ipoint.y > p1.y && ipoint.y < p2.y)         //quadrant 1
    {
        intersections.push_back(ipoint);
    }
    else if(ipoint.x < p1.x && ipoint.x > p2.x && ipoint.y < p1.y && ipoint.y > p2.y)    //quadrant 3
    {
        intersections.push_back(ipoint);
    }
    else if(ipoint.x < p1.x && ipoint.x > p2.x && ipoint.y > p1.y && ipoint.y < p2.y)    //quadrant 2
    {
        intersections.push_back(ipoint);
    }
    else if(ipoint.x > p1.x && ipoint.x < p2.x && ipoint.y < p1.y && ipoint.y > p2.y)    //quadrant 4
    {
        intersections.push_back(ipoint);
    }
    return intersections;
}

/// \brief publishes fake lidar data
void fakeLaser()
{
    //initialize
    sensor_msgs::LaserScan laser;
    std_msgs::Header head;
    const double angle_min = 0.0;                //start angle of scan
    const double angle_max = 6.28319;            //end angle of scan

    //set header
    head.frame_id = "turtle";
    head.stamp = ros::Time::now();
    laser.header = head;

    //scanner information
    laser.angle_min = angle_min;
    laser.angle_max = angle_max;
    laser.time_increment = scan_time/num_samples;
    laser.angle_increment = angle_increment;
    laser.scan_time = scan_time;
    laser.range_min = range_min;
    laser.range_max = range_max;
    laser.ranges.resize(num_samples);

    //calculate landmark positions in turtle frame
    std::vector<rigid2d::Vector2D> landmarks;
    for(int j=0; j<landmarks_world.size(); j++)
    {
        landmarks.push_back((dd.getTransform().inv())(landmarks_world[j]));
    }

    //populate data
    for(int i=0; i<num_samples; i++)
    {
        rigid2d::Vector2D p1;                                //range min point
        rigid2d::Vector2D p2;                                //range max point
        double phi = rigid2d::deg2rad(i);                    //bearing
        std::vector<rigid2d::Vector2D> intersections;        //points of intersections        ROS_ERROR_STREAM(intersections.size());

        //points on line
        p1.x = range_min*cos(phi);
        p1.y = range_min*sin(phi);
        p2.x = range_max*cos(phi);
        p2.y = range_max*sin(phi);

        //check all landmarks for intersections
        for(int j=0; j<landmarks.size(); j++)
        {
            //put landmark at (0,0)
            const rigid2d::Vector2D point1 = p1-landmarks[j];
            const rigid2d::Vector2D point2 = p2-landmarks[j];

            //check for circle intersection
            double disc = cl::discriminant(point1,point2,tube_radius);
            if (disc >= 0)
            {
                //store intersections
                rigid2d::Vector2D ipoint = cl::findIntersection(point1,point2,tube_radius)+landmarks[j];
                intersections = storeIntersections(ipoint, p1, p2, intersections);
            }
        }

        //check for wall intersections
        rigid2d::Vector2D line = cl::line(p1,p2);

        //right wall
        rigid2d::Vector2D r_wall_p1 = rigid2d::Vector2D(width/2,0);                 //points on wall
        rigid2d::Vector2D r_wall_p2 = rigid2d::Vector2D(width/2,2);
        r_wall_p1 = (dd.getTransform().inv())(r_wall_p1);                           //transform to turtle frame
        r_wall_p2 = (dd.getTransform().inv())(r_wall_p2);
        rigid2d::Vector2D r_wall = cl::line(r_wall_p1,r_wall_p2);                   //calculate line
        rigid2d::Vector2D r_intercept = cl::line_intersect(line,r_wall);            //find intercept
        intersections = storeIntersections(r_intercept, p1, p2, intersections);     //store intersections

        //left wall
        rigid2d::Vector2D l_wall_p1 = rigid2d::Vector2D(-width/2,0);                //points on wall
        rigid2d::Vector2D l_wall_p2 = rigid2d::Vector2D(-width/2,2);
        l_wall_p1 = (dd.getTransform().inv())(l_wall_p1);                           //transform to turtle frame
        l_wall_p2 = (dd.getTransform().inv())(l_wall_p2);
        rigid2d::Vector2D l_wall = cl::line(l_wall_p1,l_wall_p2);                   //calculate line
        rigid2d::Vector2D l_intercept = cl::line_intersect(line,l_wall);            //find intercept
        intersections = storeIntersections(l_intercept, p1, p2, intersections);     //store intersections

        //top wall
        rigid2d::Vector2D t_wall_p1 = rigid2d::Vector2D(0,length/2);                //points on wall
        rigid2d::Vector2D t_wall_p2 = rigid2d::Vector2D(2,length/2);
        t_wall_p1 = (dd.getTransform().inv())(t_wall_p1);                           //transform to turtle frame
        t_wall_p2 = (dd.getTransform().inv())(t_wall_p2);
        rigid2d::Vector2D t_wall = cl::line(t_wall_p1,t_wall_p2);                   //calculate line
        rigid2d::Vector2D t_intercept = cl::line_intersect(line,t_wall);            //find intercept
        intersections = storeIntersections(t_intercept, p1, p2, intersections);     //store intersections

        //bottom wall
        rigid2d::Vector2D b_wall_p1 = rigid2d::Vector2D(0,-length/2);                //points on wall
        rigid2d::Vector2D b_wall_p2 = rigid2d::Vector2D(2,-length/2);
        b_wall_p1 = (dd.getTransform().inv())(b_wall_p1);                           //transform to turtle frame
        b_wall_p2 = (dd.getTransform().inv())(b_wall_p2);
        rigid2d::Vector2D b_wall = cl::line(b_wall_p1,b_wall_p2);                   //calculate line
        rigid2d::Vector2D b_intercept = cl::line_intersect(line,b_wall);            //find intercept
        intersections = storeIntersections(b_intercept, p1, p2, intersections);     //store intersections

        const rigid2d::Vector2D zero;               //zero vector
        double distances[intersections.size()];     //array of distances
        double min_distance = 0;                    //minimum distance

        //calculate all distances
        for(int j=0; j<intersections.size(); j++)
        {
            distances[j] = cl::dist(zero,intersections[j]);
        }

        //find min distance and add noise
        min_distance = *std::min_element(distances,distances+intersections.size());
        laser.ranges[i] = min_distance+scan_noise(get_random()); 
    }
    fakeLaser_pub.publish(laser);
}

/// \brief initializes node, subscriber, publisher, parameters, and objects
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function
int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "tube_world");
    ros::NodeHandle nh;

    //initialize publishers and subscribers
    const ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 10, velCallback);
    pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("tube_locations", 10, true);
    path_pub = nh.advertise<nav_msgs::Path>("real_path", 10);
    fakeTube_pub = nh.advertise<visualization_msgs::MarkerArray>("fake_sensor", 10);
    fakeLaser_pub = nh.advertise<sensor_msgs::LaserScan>("fake_laser", 10);

    //get parameters
    double x_var;
    double w_var;
    double slip_min;
    double slip_max;
    double wheel_var;
    double tube_var;
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
    ros::param::get("/body_frame_id", body_frame_id);
    ros::param::get("/tube_var", tube_var);
    ros::param::get("/max_distance", max_distance);
    ros::param::get("/range_min",range_min);
    ros::param::get("/range_max",range_max);
    ros::param::get("/angle_increment",angle_increment);
    ros::param::get("/resolution", resolution);
    ros::param::get("/num_samples", num_samples);
    ros::param::get("/laser_var", laser_var);
    ros::param::get("/boarder_length", length);
    ros::param::get("/boarder_width", width);

    //initialize landmark locations
    for(int i = 0; i<tube_coordinates_x.size(); i++)
    {
        rigid2d::Vector2D lm;
        lm.x = tube_coordinates_x[i];
        lm.y = tube_coordinates_y[i];
        landmarks_world.push_back(lm);
    }

    // initialize noise distributions
    wheel_var = (slip_min+slip_max)/2;
    std::normal_distribution<> d(wheel_var, wheel_var-slip_min);
    std::normal_distribution<> d1(0, x_var);
    std::normal_distribution<> d2(0, w_var);
    std::normal_distribution<> d3(0, tube_var);
    std::normal_distribution<> d4(0, laser_var);
    wheel_noise = d;
    x_noise = d1;
    w_noise = d2;
    tube_noise = d3;
    scan_noise = d4;

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
    
    int i = 0;
    int j = 0;
    while(ros::ok())
    {
        publishJS();
        pubPath();
        broadcast();
        ros::spinOnce();

        i++;
        j++;
        if (i == frequency/10)
        {
            fakeSensor();   //publishes at 10hz
            fakeLaser();
            i = 0;
        }
        // if (j == frequency*scan_time)
        // {
        //     fakeLaser();   //publishes as 5hz
        //     j = 0;
        // }

        r.sleep();
    }

    return 0;
}