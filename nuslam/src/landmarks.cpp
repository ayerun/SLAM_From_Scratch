/// \file
/// \brief Implement circle regression to find landmark locations from sensor readings
///
/// PARAMETERS:
///     dist_thres: distance threshold for consecutive lidar measurements in a cluster
/// PUBLISHES:
///    /cluster_locations (visualization_msgs/MarkerArray): markers at locations of clustered data
/// SUBSCRIBES:
///     /fake_laser (sensor_msgs/LaserScan): fake lidar data
/// SERVICES:
///

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <rigid2d/rigid2d.hpp>
#include <nuslam/nuslam.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


//Global variables
static const int frequency = 200;
static double dist_thres;
static ros::Publisher cluster_pub;

/// \brief publishes marker array at cluster locations
/// \param clusters[] - array of vectors containing Vector2D points (each vector is a cluster)
/// \param num_elements - length of clusters[]
void publishClusters(std::vector<rigid2d::Vector2D> clusters[], int num_elements)
{
    visualization_msgs::MarkerArray cluster_tubes;

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

    for(int i = 0; i < num_elements; i++)
    {
        //discard clusters smaller than 3
        if(clusters[i].size() < 3)
        {
            clusters[i].clear();
        }
        else
        {
            visualization_msgs::Marker tube;
            geometry_msgs::Point pos;
            
            //set shape and color and lifetime
            tube.type = tube.CYLINDER;
            tube.color = col;
            tube.lifetime = ros::Duration(0.1);

            //set pose
            rigid2d::Vector2D avg;
            for(int j = 0; j < clusters[i].size(); j++)
            {
                avg += clusters[i][j];
            }
            avg = avg*(1.0/clusters[i].size());
            pos.x = avg.x;
            pos.y = avg.y;
            pos.z = 0;
            tube.pose.position = pos;
            tube.pose.orientation = rot;

            //set scale
            tube.scale.x = 0.0762;
            tube.scale.y = 0.0762;
            tube.scale.z = 0.1;

            tube.ns = "cluster";
            tube.header.frame_id = "turtle";
            tube.header.stamp = ros::Time::now();
            tube.id = i;

            cluster_tubes.markers.push_back(tube);
        }
    }
    cluster_pub.publish(cluster_tubes);
}

/// \brief subscriber callback that clusters laserscan data
/// \param ls - incoming laserscan message
void laserCallback(const sensor_msgs::LaserScanConstPtr &ls)
{   
    //clustering setup
    std::vector<rigid2d::Vector2D> clusters[ls->ranges.size()];     //array of vectors containing (x,y) coordinates
    int j = 0;                                                      //index for clusters
    bool flag = false;
    bool flag2 = true;

    //clustering
    for(int i = 0; i < ls->ranges.size(); i++)
    {
        //convert to xy
        rigid2d::Vector2D p_new = nuslam::rb2xy(ls->ranges[i],i);
        rigid2d::Vector2D p_old = nuslam::rb2xy(ls->ranges[i-1],i-1);

        //calculate distance
        double d = nuslam::dist(p_new,p_old);

        //if part of cluster
        if (d < dist_thres)
        {
            if(i==0)
            {
                flag = true;
            }
            clusters[j].push_back(p_new);
        }
        //if not part of cluster
        else
        {
            if(i==ls->ranges.size()-1)
            {
                flag2 = false;
            }
            j++;
        }  
    }

    //loop closure
    if (clusters[0].size()+clusters[j].size() > 2 && flag == true && flag2 == true)
    {
        clusters[j].insert(clusters[j].end(),clusters[0].begin(),clusters[0].end());
        clusters[0].clear();
    }

    publishClusters(clusters,ls->ranges.size());

    return;
}

/// \brief initializes node, subscriber, publisher, parameters, and objects
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function
int main(int argc, char** argv)
{
    //start node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;

    //initialize publishers and subscribers
    const ros::Subscriber laser_sub = nh.subscribe("fake_laser", 10, laserCallback);
    cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_locations",10);

    ros::param::get("/dist_thres", dist_thres);

    ros::Rate r(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}