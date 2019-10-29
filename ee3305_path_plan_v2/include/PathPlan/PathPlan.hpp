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
  ros::Publisher curr_pub_;
  
  cube wall_map; // It's a 3d tensor (can create as row, col, slices)
  
  // for /odom
  double pos_x_, pos_y_, ang_z_; //current robot position and orientation in euler angle
  // for /range_pub
  double max_range_ = 4.0;
  double dist_north_ = max_range_, dist_east_ = max_range_, dist_south_ = max_range_, dist_west_ = max_range_;
  
  int goal_reached_;
  
  // update the map
  
  void initializeWall();
  void setWall(int x, int y, int direction);
  void removeWall(int x, int y, int direction);
  bool hasWall(int x, int y, int direction);
  
  // callback functions
  void rangeCallback(const std_msgs::Float32MultiArray& rangeMsg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);
  
  //Djikstra Algorithm
  Mat<int> path_map_;
  vec neighbor_value_ = vec(4, fill::zeros);
  int target_x_, target_y_, target_x_prev_, target_y_prev_;
  bool path_map_initialized_ = false;
  void initializePathMap(); // set all the cell with 1000
  void setNextDestCell(); // based on the current position, find the next heading cell
  int getPathMapValue(int x, int y);
  
public:
  
  PathPlan(ros::NodeHandle& nh);
  void spin();
  void checkWall();
  void dijkstra();
  void aStar();
  
  
};


