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
#include <armadillo>


//Global variables
static const int frequency = 200;
static double dist_thres;
static double min_cluster_size;
static double max_radius;
static double max_angle_mean;
static double min_angle_mean;
static double min_standard_deviation;
static ros::Publisher cluster_pub;

void publishLandmarks(std::vector<rigid2d::Vector2D> & circles) {
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

    for(int i = 0; i < circles.size(); i++)
    {
        visualization_msgs::Marker tube;
        geometry_msgs::Point pos;
        
        //set shape and color and lifetime
        tube.type = tube.CYLINDER;
        tube.color = col;
        tube.lifetime = ros::Duration(0.1);

        //set pose
        pos.x = circles[i].x;
        pos.y = circles[i].y;
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

void classifyCircles(std::vector<std::vector<rigid2d::Vector2D>> & clusters) {
    for (int i=0; i<clusters.size(); i++) {
        std::vector<rigid2d::Vector2D> cluster = clusters[i];
        rigid2d::Vector2D p_start = cluster[0];                 //start of cluster
        rigid2d::Vector2D p_end = cluster[cluster.size()-1];    //end of cluster
        std::vector<double> angles;                             //inscribed angles
        double angle_mean = 0;                                  //mean of inscribed angles

        for (int j=1; j<cluster.size()-1; j++) {

            //make p2 the origin for p_start and p_end
            rigid2d::Vector2D p2 = cluster[j];
            rigid2d::Vector2D p1 = p_start-p2;
            rigid2d::Vector2D p3 = p_end-p2;

            //calculate inscribed angle
            double ang = rigid2d::rad2deg(rigid2d::angle(p1,p3));
            angles.push_back(ang);
            angle_mean += ang;
        }
        angle_mean /= angles.size();

        //remove clusters that exceed limits
        if (angle_mean < min_angle_mean || angle_mean > max_angle_mean) {
            clusters.erase(clusters.begin()+i);
            i--;
            continue;
        }

        //calculate standard deviation
        double sd = 0;
        for (int j=0; j<angles.size(); j++) {
            sd += pow(angles[j]-angle_mean,2);
        }
        sd = sqrt(sd/angles.size());

        //remove clusters if standard deviation is too large
        if (sd > min_standard_deviation) {
            clusters.erase(clusters.begin()+i);
            i--;
        }
    }
}

void circleRegression(std::vector<std::vector<rigid2d::Vector2D>> & clusters) {
    //store circle parameters
    std::vector<rigid2d::Vector2D> circles;

    for (int i=0; i<clusters.size(); i++) {
        std::vector<rigid2d::Vector2D> cluster = clusters[i];
        int n = cluster.size();

        //compute means
        double x_mean = 0;
        double y_mean = 0;
        for (int j=0; j<n; j++) {
            x_mean += cluster[j].x;
            y_mean += cluster[j].y;
        }

        //find centroid
        double x_cen = x_mean/n;
        double y_cen = y_mean/n;

        //shift points so centroid is at origin
        //compute z mean
        double z_mean = 0;
        std::vector<double> z_list;
        std::vector<double> x_list;
        std::vector<double> y_list;
        for (int j=0; j<n; j++) {
            x_list.push_back(cluster[j].x - x_cen);
            y_list.push_back(cluster[j].y - y_cen);

            double z_val = pow(x_list[j],2) + pow(y_list[j],2);
            z_list.push_back(z_val);
            z_mean += z_val;
        }
        double z_cen = z_mean/n;

        //Form Z matrix
        arma::mat col1(z_list);
        arma::mat col2(x_list);
        arma::mat col3(y_list);
        arma::mat col4(n,1,arma::fill::ones);
        arma::mat Z = arma::join_rows(col1,col2,col3,col4);

        //Form momemt matrix
        arma::mat M = (1/n)*arma::trans(Z)*Z;

        //Form constraint matrix
        arma::mat H = { {8*z_cen, 0, 0, 2},
                        {0,       1, 0, 0},
                        {0,       0, 1, 0},
                        {2,       0, 0, 0} };
        arma::mat Hinv = {  {0, 0, 0, 0.5},
                            {0, 1, 0, 0},
                            {0, 0, 1, 0},
                            {0.5, 0, 0, -2*z_cen}  };

        //SVD of Z
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd(U,s,V,Z);

        //Solve for A
        arma::vec A;
        if (s(3) < 1e-12) A = V.col(3); 
        else {
            arma::mat Y = V*arma::diagmat(s)*V.t();
            arma::mat Q = Y*Hinv*Y;
            arma::vec eigval;
            arma::mat eigvec;
            arma::eig_sym(eigval, eigvec, Q);

            int index = 0;
            double val = INT_MAX;
            for (int i = 0; i<eigval.size();i++){
                if (eigval[i]<val & eigval[i] > 0){
                    val = eigval[i];
                    index = i;
                }
            }

            arma::vec Astar = eigvec.col(index);
            A = arma::solve(Y,Astar);
        }

        // Calculate equation for circle
        double a = -A(1)/(2*A(0))+x_cen;  //x center
        double b = -A(2)/(2*A(0))+y_cen;  //y center
        double R = sqrt((pow(A(1),2) + pow(A(2),2) -4*A(0)*A(3)) / (4*pow(A(0),2)));    //radius

        // Ignore circles with large radii
        if (R<max_radius) circles.push_back(rigid2d::Vector2D(a,b));
    }
    publishLandmarks(circles);
    return;
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
            if (cluster.size() >= min_cluster_size) clusters.push_back(cluster);
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
    
    classifyCircles(clusters);
    circleRegression(clusters);
    return;
}

/// \brief initializes node, subscriber, publisher, parameters, and objects
/// \param argc - initialization arguement
/// \param argv - initialization arguement
/// \return 0 at end of function
int main(int argc, char** argv) {
    //start node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;

    //initialize publishers and subscribers
    const ros::Subscriber laser_sub = nh.subscribe("fake_laser", 10, laserCallback);
    cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_locations",10);

    //get clustering parameters
    ros::param::get("/dist_thres", dist_thres);
    ros::param::get("/min_cluster_size", min_cluster_size);
    ros::param::get("/max_radius", max_radius);
    ros::param::get("/max_angle_mean", max_angle_mean);
    ros::param::get("/min_angle_mean", min_angle_mean);
    ros::param::get("/min_standard_deviation", min_standard_deviation);

    ros::Rate r(frequency);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}