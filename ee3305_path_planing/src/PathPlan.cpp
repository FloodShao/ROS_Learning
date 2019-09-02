#include <ros/ros.h>
#include "PathPlan/PathPlan.hpp"
#include "PreDefine.hpp"
#include <math.h>

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
  
  checkWall();
  dijkstra();
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
  pos_y_ = odomMsg->pose.pose.position.x;
  pos_x_ = -(odomMsg->pose.pose.position.y);
  
  //ROS_INFO("Received /odom position (%f, %f).", pos_x_, pos_y_);
  
  float q_x = odomMsg->pose.pose.orientation.x;
  float q_y = odomMsg->pose.pose.orientation.y;
  float q_z = odomMsg->pose.pose.orientation.z;
  float q_w = odomMsg->pose.pose.orientation.w;
  ang_z_ = -atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_x*q_x + q_y*q_y)); //because in the world frame, there is a 90 degree transformation
  
  checkWall();
  dijkstra();
  
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
  //mind that the map we draw has a different orientation with the cell matrix 
  wall_map = cube(GRID_SIZE, GRID_SIZE, 4, fill::zeros); //each cell has four direction, with 0 on one direction means open on that direction
  
  for(int row = 0; row < GRID_SIZE; row++){
    for(int col = 0; col < GRID_SIZE; col++){
      if(row == 0){
	wall_map(row, col, WEST) = WALL;
      }
      if(row == GRID_SIZE-1){
	wall_map(row, col, EAST) = WALL;
      }
      if(col == 0){
	wall_map(row, col, SOUTH) = WALL;
      }
      if(col == GRID_SIZE-1){
	wall_map(row, col, NORTH) = WALL;
      }
    }
  }
}

/*
 * path map related
 */
void PathPlan::initialize_path_map()
{
  path_map = Mat<int>(GRID_SIZE, GRID_SIZE); //set all the path value as inf
  path_map.fill(1000);
  path_map(GOAL_X, GOAL_Y) = 0;
  path_map_initialized = true;
  //update the path plan with current obstacles
  dijkstra();
}

void PathPlan::dijkstra()
{
  // make sure the goal cell has been put in the path_map
  if(!path_map_initialized) initialize_path_map();
  
  mat visited_map(GRID_SIZE, GRID_SIZE, fill::zeros);
  mat is_queued(GRID_SIZE, GRID_SIZE, fill::zeros);
  
  std::vector<pair<int, int>> reached_queue;
  reached_queue.push_back(pair<int, int>(GOAL_X, GOAL_Y));
  
  int nVisited = 0;
  int nCells = GRID_SIZE * GRID_SIZE;
  int mdist_from_goal_node = 0;
  pair<int, int> node;
  
  int x_queue, y_queue; //tmp coord in queue
  
  while(nVisited < nCells){
    int min_dist = 1000;
    int queue_length = reached_queue.size();
    ROS_INFO("queue length: %d", queue_length);
    
    for(int i = 0; i<queue_length; i++){
      pair<int, int> tmp = reached_queue[i];
      //find the minimum goal distance node that hasn't been visited
      if(visited_map.at(tmp.first, tmp.second) == 0){
	if(path_map(tmp.first, tmp.second) < min_dist){
	  min_dist = path_map.at(tmp.first, tmp.second);
	  node = tmp;
	}
      }
    }
    
    visited_map(node.first, node.second) = 1; //this node has been reached
    visited_map.print();
    nVisited++;
    ROS_INFO("Add %d nodes. min_dist: %d", nVisited, min_dist);
    
    // extend to adjacent cells to this node
    for(int direction = NORTH; direction <= WEST; direction++){
      
	if(!hasWall(node.first, node.second, direction)){
	  // there is no wall on that direction, and the adjacent node hasn't been put in the queue_length
	  
	  if(direction == NORTH && node.second+1 < GRID_SIZE){
	    x_queue = node.first; y_queue = node.second+1;
	  }
	  if(direction == EAST && node.first+1 < GRID_SIZE){
	    x_queue = node.first+1; y_queue = node.second;
	  }
	  if(direction == SOUTH && node.second-1 >= 0){
	    x_queue = node.first; y_queue = node.second-1;
	  }
	  if(direction == WEST && node.first-1 >= 0){
	    x_queue = node.first-1; y_queue = node.second;
	  }
	  
	  if(!is_queued(x_queue, y_queue)){
	    // put the adjacent cell in the queue
	    reached_queue.push_back(pair<int, int>(x_queue, y_queue));
	    is_queued(x_queue, y_queue) = 1;
	  }
	  
	  // update the path_map, update the shortest path to goal
	  path_map(x_queue, y_queue) = min(path_map(x_queue, y_queue), min_dist+1);
	  
	  ROS_INFO("%d no wall. (%d, %d) path_map is %d", direction, x_queue, y_queue, path_map(x_queue, y_queue));
	}
	
    }
    // test print the path_map
    path_map.print();
  }
  

}














