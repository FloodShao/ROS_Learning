/* This node returns an array with distances from the scan topic:
The leftmost measured distance, middle distance and rightmost distance in
the new topic range_finder/distances which is a message of type
Float32MultiArray. The distances is stored in the "data" sub-message
within the topic, as [left, middle, right]
 If the message from /scan returns "nan", this node will either set the distance
 to 0.01 (wall) or 10 (distance too far for the sensor to see).
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <armadillo>
#include <std_msgs/Float32MultiArray.h>

using namespace arma;

class RangeFinder{
private:
    ros::Subscriber sub_scan;
    ros::Publisher pub_range_finder;
    std::vector<float> scan_vector;
    bool initialized;
    double max_dist, min_dist;
    float scan_left, scan_middle, scan_right;
    float scan_left_prev, scan_middle_prev, scan_right_prev;
public:
    RangeFinder(ros::NodeHandle &nh);
    void callback_range_finder(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
};


RangeFinder::RangeFinder(ros::NodeHandle &nh) {
    //sub_scan = nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&BotController::processLaserScan,this);
    sub_scan = nh.subscribe("/scan", 1, &RangeFinder::callback_range_finder, this);
    pub_range_finder = nh.advertise<std_msgs::Float32MultiArray>("/range_finder/distances",1);
    max_dist = 10;
    min_dist = 0.01;
    scan_left_prev = 3;
    scan_middle_prev = 3;
    scan_right_prev = 3;
    initialized = false;
}

// Find the range(distance to obstacle)to the wall
void RangeFinder::callback_range_finder(const sensor_msgs::LaserScan::ConstPtr& scanMsg){

    scan_vector = scanMsg->ranges;
    int scan_vector_middle_index = (int)round(scan_vector.size() / 2.0);
    scan_right = scan_vector.front();
    scan_middle = scan_vector[scan_vector_middle_index];
    scan_left = scan_vector.back();

    // Determine the distance to the left wall
    if (isnanf(scan_left)){
        if(!initialized){
            scan_left = max_dist;
        }
        else if(scan_left_prev > 2){
            scan_left = max_dist;
        }
        else if(scan_left_prev <= 2){
            scan_left = min_dist;
        }
    }

    // Determine the distance to the middle wall
    if (isnanf(scan_middle)){
        if(!initialized){
            scan_middle = max_dist;
        }
        else if(scan_middle_prev > 2){
            scan_middle = max_dist;
        }
        else if(scan_middle_prev <= 2){
            scan_middle = min_dist;
        }
    }

    // Determine the distance to the right wall
    if (isnanf(scan_right)){
        if(!initialized){
            scan_right = max_dist;
        }
        else if(scan_right_prev > 2){
            scan_right = max_dist;
        }
        else if(scan_left_prev <= 2){
            scan_right = min_dist;
        }
    }

    if(!initialized){
        initialized = true;
    }

    scan_left_prev = scan_left;
    scan_middle_prev = scan_middle;
    scan_right_prev = scan_right;

    // Send the data to the path_planner node to update the wall map
    std_msgs::Float32MultiArray cmd_distances;
    cmd_distances.data.clear();
    cmd_distances.data.push_back(scan_left);
    cmd_distances.data.push_back(scan_middle);
    cmd_distances.data.push_back(scan_right);

    pub_range_finder.publish(cmd_distances);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "range_finder");
    ros::NodeHandle nh;
    RangeFinder dl(nh);

    ros::spin();
    return 0;
}
