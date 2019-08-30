#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

using namespace std;

class RangeDetect{

private:
  
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher range_detect_pub_;
  double scan_front_, scan_left_, scan_right_;
  vector<float> scan_data_;
  
  double max_dist_ = 10;
  double min_dist_ = 0.01;
  double scan_front_prev_ = 3;
  double scan_left_prev_ = 3;
  double scan_right_prev_ = 3;
  bool initialized_ = false;
  
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
  
public:
  
  RangeDetect(ros::NodeHandle& nh);
  
};
