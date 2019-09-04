#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

enum{GOAL_NOT_REACH = 0, GOAL_REACH = 1};
class BotControl{
  
private:
  
  ros::NodeHandle nh_;
  ros::Subscriber target_sub_;
  ros::Subscriber curr_sub_;
  ros::Publisher control_pub_;
  
  void targetCallBack(const geometry_msgs::PointConstPtr& target_msg);
  void currCallBack(const geometry_msgs::PointConstPtr& curr_msg);
  
  int target_x_, target_y_, goal_reached_;
  double pos_x_, pos_y_, heading_;
  double error_pos_, error_pos_prev_, error_heading_, error_heading_prev_;
  
  double Kp_x, Ki_x, Kd_x, Kp_a, Ki_a, Kd_a;
  void controlPub();
  
  double PI = 3.1415926;
  double dt = 2;
  
  double max_vel = 0.7;
  double max_ang = PI;
  
  
public:
  
  BotControl(ros::NodeHandle& nh);
  bool loadParam();
  void spin();
  
};