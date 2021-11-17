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

void publishClusters(std::vector<std::vector<rigid2d::Vector2D>> & clusters) {
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

    for(int i = 0; i < clusters.size(); i++)
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
    cluster_pub.publish(cluster_tubes);
}

/// \brief subscriber callback that clusters laserscan data
/// \param ls - incoming laserscan message
void laserCallback(const sensor_msgs::LaserScanConstPtr &ls) {   
    //clustering setup
    std::vector<std::vector<rigid2d::Vector2D>> clusters;     //array of vectors containing (x,y) coordinates
    std::vector<rigid2d::Vector2D> cluster;

    for (int i=1; i<ls->ranges.size(); i++) {
        
        //convert to xy
        rigid2d::Vector2D p_new = nuslam::rb2xy(ls->ranges[i],i);
        rigid2d::Vector2D p_old = nuslam::rb2xy(ls->ranges[i-1],i-1);

        //calculate distance
        double d = nuslam::dist(p_new,p_old);

        //part of cluster
        if (d < dist_thres) {
            if (i == 1) {
                cluster.push_back(p_old);
            }
            cluster.push_back(p_new);
        }

        //not part of cluster
        else {
            if (cluster.size() >= 3) clusters.push_back(cluster);
            cluster.clear();
        }

        //loop closure
        if (i+1 == ls->ranges.size()) {
            rigid2d::Vector2D first = clusters[0][0];           //first clustered point

            //ensure first clustered point is first point in laser scan
            if (first == nuslam::rb2xy(ls->ranges[0],0)) {
                double d2 = nuslam::dist(p_new,first);

                //cluster
                if (d2 < dist_thres) {
                    
                    //check size of last cluster
                    if (cluster.size() == 0) clusters[0].push_back(p_new);
                    else {
                        clusters[0].insert(clusters[0].end(),cluster.begin(),cluster.end());
                        cluster.clear();
                    }
                }
            }
        }

    }
    
    publishClusters(clusters);
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