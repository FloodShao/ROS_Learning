#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> //input from range_finder
#include <nav_msgs/Odometry.h> //input from odometry
#include <geometry_msgs/Point.h>  
#include <armadillo>
#include <math.h>
#include <stdio.h>

using namespace std;
using namespace arma;

class PathPlan{
private:
  
  ros::NodeHandle nh_;
  ros::Subscriber range_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher target_pub_;
  ros::Publisher control_pub_;
  
  
  cube wall_map; // It's a 3d tensor (can create as row, col, slices)
  
  double pos_x_, pos_y_, ang_z_; //current robot position and orientation in euler angle
  double dist_left_, dist_front_, dist_right_;
  
  // update the map
  void checkWall();
  void initializeWall();
  void setWall(int x, int y, int direction);
  void removeWall(int x, int y, int direction);
  bool hasWall(int x, int y, int direction);
  
  // callback functions
  void rangeCallback(const std_msgs::Float32MultiArray& rangeMsg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);
  
  //Djikstra Algorithm
  mat path_map;
  void initialize_path_map();
  void update_path_map();
  
public:
  
  PathPlan(ros::NodeHandle& nh);
  void spin();
  
};


