#include "RangeDetect/RangeDetect.hpp"

RangeDetect::RangeDetect(ros::NodeHandle& nh)
{
  nh_ = nh;
  scan_sub_ = nh_.subscribe("/scan", 1, &RangeDetect::scanCallback, this);
  range_detect_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/range_detect/distances", 1);

  ROS_INFO("range_detect_node initialized successfully");
}

/*
 * scanCallback Function
 * @param: scanMsg 
 * @remark: this function is intended to be applied on husky_simulator, the laser scan range (deg) is -145 to 145
 */
void RangeDetect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
  scan_data_ = scanMsg->ranges;
  int scan_size = scan_data_.size();
  int scan_mid_idx = 0;
  int scan_left_idx = scan_mid_idx + scan_size / 4; // 90 degrees to the front
  int scan_right_idx = scan_mid_idx + scan_size * 3 / 4;
  
  scan_left_ = scan_data_[scan_left_idx];
  scan_right_ = scan_data_[scan_right_idx];
  scan_front_ = scan_data_[scan_mid_idx];
  
  if(isinf(scan_left_)){ // left detects nothing
    if(!initialized_){ //haven't initialized
      scan_left_ = max_dist_;
    //} else if(scan_left_prev_ > 2){
    //  scan_left_ = max_dist_;
    } else{ //the distance is too small to be detected
      scan_left_ = max_dist_;
    }
  }
  
  if(isinf(scan_right_)){ // left detects nothing
    if(!initialized_){ //haven't initialized
      scan_right_ = max_dist_;
    //} else if(scan_right_prev_ > 2){
    //  scan_right_ = max_dist_;
    } else{ //the distance is too small to be detected
      scan_right_ = max_dist_;
    }
    
  }
  
  if(isinf(scan_front_)){ // left detects nothing
    if(!initialized_){ //haven't initialized
      scan_front_ = max_dist_;
    //} else if(scan_front_prev_ > 2){
    //  scan_front_ = max_dist_;
    } else{ //the distance is too small to be detected
      scan_front_ = max_dist_;
    }
  }
  
  if(!initialized_) initialized_ = true;
  
  //update the previous scan data
  scan_left_prev_ = scan_left_;
  scan_right_prev_ = scan_right_;
  scan_front_prev_ = scan_front_;
  
  //construct the publish msg;
  std_msgs::Float32MultiArray distance_pub;
  distance_pub.data.clear();
  distance_pub.data.push_back(scan_left_);
  distance_pub.data.push_back(scan_front_);
  distance_pub.data.push_back(scan_right_);
  range_detect_pub_.publish<std_msgs::Float32MultiArray>(distance_pub);
  
  ROS_INFO("range_detect_node: left: %f front: %f right: %f", scan_left_, scan_front_, scan_right_);
}



