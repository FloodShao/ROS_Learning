#include <ros/ros.h>
#include "PathPlan/PathPlan.hpp"
#include "PreDefine.hpp"

/*
 * Constructor
 */

PathPlan::PathPlan(ros::NodeHandle& nh)
{
  nh_ = nh;
  range_sub_ = nh_.subscribe("/range_detect/distances", 1, &PathPlan::rangeCallback, this);
  odom_sub_ = nh_.subscribe("/husky_velocity_controller/odom", 1, &PathPlan::odomCallback, this);
  target_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/target", 1);
  control_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/control", 1);
  
  initializeWall();
  initialize_path_map();
  
  ROS_INFO("PathPlan node initialized successfully!");
}

void PathPlan::spin()
{
  ros::Rate loop_rate(1.0/dt);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}


/*
 * Callback Function
 */

void PathPlan::odomCallback(const nav_msgs::OdometryConstPtr& odomMsg)
{
  
  //use the first pose of husky as the original point, with front pointing to y axis
  pos_y_ = odomMsg->pose.pose.position.x + 0.5; //0.5 is the original coordinate of husky
  pos_x_ = -(odomMsg->pose.pose.position.y) + 0.5;
  
  //ROS_INFO("Received /odom position (%f, %f).", pos_x_, pos_y_);
  
  float q_x = odomMsg->pose.pose.orientation.x;
  float q_y = odomMsg->pose.pose.orientation.y;
  float q_z = odomMsg->pose.pose.orientation.z;
  float q_w = odomMsg->pose.pose.orientation.w;
  ang_z_ = -atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_x*q_x + q_y*q_y)); //because in the world frame, there is a 90 degree transformation
  
  checkWall();
  
  
  
}

void PathPlan::rangeCallback(const std_msgs::Float32MultiArray& rangeMsg)
{
  dist_left_ = rangeMsg.data[0];
  dist_front_ = rangeMsg.data[1];
  dist_right_ = rangeMsg.data[2];
  
  //ROS_INFO("Received dist: %f, %f, %f.", dist_left_, dist_front_, dist_right_);

}



/*
 * Wall Map related
 */
void PathPlan::checkWall()
{
  //in world frame
  int pos_x_int = (int)floor(pos_x_);
  int pos_y_int = (int)floor(pos_y_);
  double ang_z_deg = ang_z_ * 180 / PI;
  
  ROS_INFO("In check wall, cell(%d, %d), ang_z_=%f", pos_x_int, pos_y_int, ang_z_deg);
  
  int direction = UNDEFINED; //it's the robot heading direction
  
  if(ang_z_deg > -MAX_ANGLE_DEVIATION && ang_z_deg < MAX_ANGLE_DEVIATION){
    direction = NORTH;
  }
  if(ang_z_deg > (90-MAX_ANGLE_DEVIATION) && ang_z_deg < (90+MAX_ANGLE_DEVIATION)){
    direction = EAST;
  }
  if(ang_z_deg > (180-MAX_ANGLE_DEVIATION) || ang_z_deg < (-180+MAX_ANGLE_DEVIATION)){
    direction = SOUTH;
  }
  if(ang_z_deg > (-90-MAX_ANGLE_DEVIATION) && ang_z_deg < (-90+MAX_ANGLE_DEVIATION)){
    direction = WEST;
  }
  
  if(dist_front_ < WALL_DETECT_DIST && direction != UNDEFINED){
    setWall(pos_x_int, pos_y_int, direction);
  }
  if(dist_front_ > OPEN_DETECT_DIST && direction != UNDEFINED){
    removeWall(pos_x_int, pos_y_int, direction);
  }
  
  if(dist_left_ < WALL_DETECT_DIST && direction != UNDEFINED){
    if(0 > direction-1) setWall(pos_x_int, pos_y_int, WEST);
    else setWall(pos_x_int, pos_y_int, direction-1);
  }
  
  if(dist_left_ > OPEN_DETECT_DIST && direction != UNDEFINED){
    if(0 > direction-1) removeWall(pos_x_int, pos_y_int, WEST);
    else removeWall(pos_x_int, pos_y_int, direction-1);
  }
  
  if(dist_right_ < WALL_DETECT_DIST && direction !=UNDEFINED){
    if(3 < direction+1) setWall(pos_x_int, pos_y_int, NORTH);
    else setWall(pos_x_int, pos_y_int, direction+1);
  }
  
  if(dist_right_ > OPEN_DETECT_DIST && direction !=UNDEFINED){
    if(3 < direction+1) removeWall(pos_x_int, pos_y_int, NORTH);
    else removeWall(pos_x_int, pos_y_int, direction+1);
  }
  
  
}


void PathPlan::setWall(int x, int y, int direction)
{
  wall_map(x, y, direction) = WALL;
  ROS_INFO("Set Wall for Cell(%d, %d, %d).", x, y, direction);
  
  // fill wall to adjacent cells
  if(direction == NORTH){
    if(y+1 < GRID_SIZE) wall_map(x, y+1, SOUTH) = WALL;
    return;
  }
  if(direction == SOUTH){
    if(y-1 >= 0) wall_map(x, y-1, NORTH) = WALL;
    return;
  }
  if(direction == WEST){
    if(x-1 >= 0) wall_map(x-1, y, EAST) = WALL;
    return;
  }
  if(direction == EAST){
    if(x+1 < GRID_SIZE) wall_map(x+1, y, WEST) = WALL;
    return;
  }
  
}


bool PathPlan::hasWall(int x, int y, int direction)
{
  if(wall_map(x, y, direction) == WALL) {
    ROS_INFO("Cell(%d, %d, %d) has Wall in the map!", x, y, direction);
    return true;
  }
  else return false;
}

void PathPlan::removeWall(int x, int y, int direction)
{
  wall_map(x, y, direction) == OPEN;
  ROS_INFO("Remove Wall for Cell(%d, %d, %d).", x, y, direction);
}


void PathPlan::initializeWall()
{
  wall_map = cube(GRID_SIZE, GRID_SIZE, 4, fill::zeros); //each cell has four direction, with 0 on one direction means open on that direction
  
  for(int row = 0; row < GRID_SIZE; row++){
    for(int col = 0; col < GRID_SIZE; col++){
      if(row == 0){
	wall_map(row, col, SOUTH) = WALL;
      }
      if(row == GRID_SIZE-1){
	wall_map(row, col, NORTH) = WALL;
      }
      if(col == 0){
	wall_map(row, col, WEST) = WALL;
      }
      if(col == GRID_SIZE-1){
	wall_map(row, col, EAST) = WALL;
      }
    }
  }
}

/*
 * path map related
 */
void PathPlan::initialize_path_map()
{
  path_map.zeros(GRID_SIZE, GRID_SIZE);
  update_path_map();
}

void PathPlan::update_path_map()
{
  mat visited_map(GRID_SIZE, GRID_SIZE, fill::zeros);
  
  

}










