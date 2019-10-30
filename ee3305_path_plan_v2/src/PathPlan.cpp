#include <ros/ros.h>
#include "PathPlan/PathPlan.hpp"
#include "PreDefine.hpp"
#include <math.h>

/*
 * Constructor
 */

/*
 * testing the Astar algorithm
 */
typedef struct {
  int x, y, f, g, h;
} node;

struct compareNode{
  bool operator() (const node &lhs, const node &rhs) const {
    return lhs.f < rhs.f;
  }
};

static int manhattenDist(node start, node end){
  int x = abs(start.x - end.x);
  int y = abs(start.y - end.y);
  return x+y;
}


PathPlan::PathPlan(ros::NodeHandle& nh)
{
  nh_ = nh;
  range_sub_ = nh_.subscribe("/range_pub", 1, &PathPlan::rangeCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &PathPlan::odomCallback, this);
  target_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/target", 1);
  curr_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/curr", 1);
  
  target_x_ = 0;
  target_y_ = 0;
  target_x_prev_ = 0;
  target_y_prev_ = 0;
  
  goal_reached_ = GOAL_NOT_REACH;
  
  initializeWall();
  initializePathMap();
  
  ROS_INFO("PathPlan node initialized successfully!");
  
  /* for test purpose on dijkstra
  setWall(0, 0, EAST);
  setWall(0, 1, EAST);
  setWall(0, 2, EAST);
  setWall(0, 3, EAST);
  setWall(0, 4, EAST);
  setWall(0, 5, EAST);
  setWall(0, 6, EAST);
  setWall(0, 7, EAST);
  wall_map.print();
  dijkstra();
  */
  //wall_map.print();
  
  
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
  pos_y_ = odomMsg->pose.pose.position.y;
  pos_x_ = odomMsg->pose.pose.position.x;
  
  //ROS_INFO("Received /odom position (%f, %f).", pos_x_, pos_y_);
  
  float q_x = odomMsg->pose.pose.orientation.x;
  float q_y = odomMsg->pose.pose.orientation.y;
  float q_z = odomMsg->pose.pose.orientation.z;
  float q_w = odomMsg->pose.pose.orientation.w;
  ang_z_ = atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_z*q_z + q_y*q_y)); //because in the world frame, there is a 90 degree transformation
  //mind that ang_z_ is with respect to the x axis of the world frame
  
  
}

void PathPlan::rangeCallback(const std_msgs::Float32MultiArray& rangeMsg)
{
  dist_north_ = rangeMsg.data[0];
  dist_east_ = rangeMsg.data[1];
  dist_south_ = rangeMsg.data[2];
  dist_west_ = rangeMsg.data[3];
  
  //ROS_INFO("range: N:%f, E:%f, S:%f, W:%f", dist_north_, dist_east_, dist_south_, dist_west_);
  
  // check the wall distribution and update the path map every time the robot localize itself
  checkWall();
  //wall_map.print();

  // Using dijkstra algorithm
  dijkstra();

  // Using Astar algorithm
  // aStar();

  if(!goal_reached_){
    setNextDestCell();
  }
  
  geometry_msgs::Point target_point;
  geometry_msgs::Point curr_position;
  
  target_point.x = target_x_;
  target_point.y = target_y_;
  target_point.z = goal_reached_;
  
  curr_position.x = pos_x_;
  curr_position.y = pos_y_;
  curr_position.z = ang_z_*180/PI; //in degree
  
  ROS_INFO("target: X: %d, Y: %d", target_x_, target_y_);
  ROS_INFO("ang_z_: %f", ang_z_ /PI*180);
  
  target_pub_.publish<geometry_msgs::Point>(target_point);
  curr_pub_.publish<geometry_msgs::Point>(curr_position);

}


/*
 * Wall Map related
 */
void PathPlan::checkWall()
{
  //in world frame
  int pos_x_int = (int)floor(pos_x_);
  int pos_y_int = (int)floor(pos_y_);
  
  // in north direction
  if(dist_north_ < WALL_DETECT_DIST){
    //ROS_INFO("set N, %f, %f", dist_north_, WALL_DETECT_DIST);
    setWall(pos_x_int, pos_y_int, NORTH);
  } 
  if(dist_north_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, NORTH);
  }
  if(dist_north_ < SECOND_WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int+1, NORTH);
  }
  if(dist_north_ > SECOND_OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int+1, NORTH);
  }
  
  //in east direction
  if(dist_east_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, EAST);
  } 
  if(dist_east_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, EAST);
  }
  if(dist_east_ < SECOND_WALL_DETECT_DIST){
    setWall(pos_x_int+1, pos_y_int, EAST);
  }
  if(dist_east_ > SECOND_OPEN_DETECT_DIST){
    removeWall(pos_x_int+1, pos_y_int, EAST);
  }
  
  //in south direction
  if(dist_south_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, SOUTH);
  } 
  if(dist_south_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, SOUTH);
  }
  if(dist_south_ < SECOND_WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int-1, SOUTH);
  }
  if(dist_south_ > SECOND_OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int-1, SOUTH);
  }

  //in west direction
  if(dist_west_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, WEST);
  } 
  if(dist_west_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, WEST);
  }
  if(dist_west_ < SECOND_WALL_DETECT_DIST){
    setWall(pos_x_int-1, pos_y_int, WEST);
  }
  if(dist_west_ > SECOND_OPEN_DETECT_DIST){
    removeWall(pos_x_int-1, pos_y_int, WEST);
  }

  
}


void PathPlan::setWall(int x, int y, int direction)
{
  if(x<0 || y <0 || direction <0 || x>= GRID_SIZE || y >= GRID_SIZE){
    ROS_ERROR("setWall function input with illegal: x:%d, y:%d, direction:%d", x, y, direction);
    return;
  }
  wall_map(x, y, direction) = WALL;
  ROS_INFO("Wall in %d", direction);
  
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
  if(x<0 || y<0 || direction<0 || x>= GRID_SIZE || y >= GRID_SIZE){
    ROS_ERROR("hasWall get an illegal input: x:%d, y:%d, direction:%d", x, y, direction);
    return true; //return with positive wall
  }
  if(wall_map(x, y, direction) == WALL) {
    //ROS_INFO("Cell(%d, %d, %d) has Wall in the map!", x, y, direction);
    return true;
  }
  else return false;
}

void PathPlan::removeWall(int x, int y, int direction)
{
  if(x<0 || y<0 || direction<0 || x>= GRID_SIZE || y >= GRID_SIZE){
    ROS_ERROR("removeWall get an illegal input: x:%d, y:%d, direction:%d", x, y, direction);
    return;
  }
  wall_map(x, y, direction) = OPEN;
  //ROS_INFO("Remove Wall for Cell(%d, %d, %d).", x, y, direction);
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
void PathPlan::initializePathMap()
{
  path_map_ = Mat<int>(GRID_SIZE, GRID_SIZE); //set all the path value as inf
  path_map_.fill(1000);
  path_map_(GOAL_X, GOAL_Y) = 0;
  path_map_initialized_ = true;
  //update the path plan with current obstacles
  //dijkstra();
}

void PathPlan::dijkstra()
{
  // make sure the goal cell has been put in the path_map
  initializePathMap();
  
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
    //ROS_INFO("queue length: %d", queue_length);
    
    for(int i = 0; i<queue_length; i++){
      pair<int, int> tmp = reached_queue[i];
      //find the minimum goal distance node that hasn't been visited
      if(visited_map.at(tmp.first, tmp.second) == 0){
	if(path_map_(tmp.first, tmp.second) < min_dist){
	  min_dist = path_map_.at(tmp.first, tmp.second);
	  node = tmp;
	}
      }
    }
    
    visited_map(node.first, node.second) = 1; //this node has been reached
    //visited_map.print();
    nVisited++;
    //ROS_INFO("Add %d nodes. min_dist: %d", nVisited, min_dist);
    
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
	  
    if(x_queue >= 0 && x_queue < GRID_SIZE && y_queue>=0 && y_queue < GRID_SIZE){
      if(!is_queued(x_queue, y_queue)){
        // put the adjacent cell in the queue
        reached_queue.push_back(pair<int, int>(x_queue, y_queue));
        is_queued(x_queue, y_queue) = 1;
      }
      
      // update the path_map, update the shortest path to goal
      path_map_(x_queue, y_queue) = min(path_map_(x_queue, y_queue), min_dist+1);
      
    }
	  
	  //ROS_INFO("%d no wall. (%d, %d) path_map is %d", direction, x_queue, y_queue, path_map(x_queue, y_queue));
	} 
    }
    //test print the path_map
    //path_map_.print();
    
  }
  
}


void PathPlan::setNextDestCell()
{
  //locate the current cell
  int pos_x_int = (int)floor(pos_x_);
  int pos_y_int = (int)floor(pos_y_);
  
  if(pos_x_int == GOAL_X && pos_y_int == GOAL_Y){
    goal_reached_ = GOAL_REACH;
    ROS_INFO("Goal Reach!, X:%d, Y:%d", GOAL_X, GOAL_Y);
    return;
  }
  
  ROS_INFO("pos_x_int:%d, pos_y_int:%d", pos_x_int, pos_y_int);
  
  neighbor_value_(NORTH) = getPathMapValue(pos_x_int, pos_y_int+1);
  neighbor_value_(EAST) = getPathMapValue(pos_x_int+1, pos_y_int);
  neighbor_value_(SOUTH) = getPathMapValue(pos_x_int, pos_y_int-1);
  neighbor_value_(WEST) = getPathMapValue(pos_x_int-1, pos_y_int);
  
  //ROS_INFO("N:%f, E:%f, S:%f, W:%f", neighbor_value_(NORTH), neighbor_value_(EAST), neighbor_value_(SOUTH), neighbor_value_(WEST));
  
  int min_heading = UNDEFINED;
  int min_path_value = 1000;
  
  
  
  for(int direction = NORTH; direction <= WEST; direction++){
    if(!hasWall(pos_x_int, pos_y_int, direction)){
      //find the path with minimum path value and without a obstacle
      if(neighbor_value_(direction) <= min_path_value){
	
	min_path_value = neighbor_value_(direction);
	min_heading = direction;
	
      }
      
      /*
      if(neighbor_value_(direction) == min_path_value){
	if(min_heading == NORTH){next_x = pos_x_int; next_y = pos_y_int+1;}
	if(min_heading == EAST){next_x = pos_x_int+1; next_y = pos_y_int;}
	if(min_heading == SOUTH){next_x = pos_x_int; next_y = pos_y_int-1;}
	if(min_heading == WEST){next_x = pos_x_int-1; next_y = pos_y_int;}
	if(dis_from_goal > (GOAL_X - next_x) * (GOAL_X - next_x) + (GOAL_Y - next_y) * (GOAL_Y - next_y)){
	  min_heading = direction;
	  min_path_value = neighbor_value_(direction);
	}
	
      }
      */
      
    }
  }

  ROS_INFO("min_head: %d", min_heading);
  
  if(min_heading < 0){ //haven't find correct path, then go back
    target_x_ = target_x_prev_;
    target_y_ = target_y_prev_;
  }else{ //update the next target destination
    target_x_prev_ = target_x_;
    target_y_prev_ = target_y_;
    
    if(min_heading == NORTH){target_x_ = pos_x_int; target_y_ = pos_y_int+1;}
    if(min_heading == EAST){target_x_ = pos_x_int+1; target_y_ = pos_y_int;}
    if(min_heading == SOUTH){target_x_ = pos_x_int; target_y_ = pos_y_int-1;}
    if(min_heading == WEST){target_x_ = pos_x_int-1; target_y_ = pos_y_int;}
    
  }

  int current_value = getPathMapValue(pos_x_int, pos_y_int);
  
  ROS_INFO("current_value: %d, neighbor_value_: %d. Next destination: (%d, %d)", current_value, min_path_value, target_x_, target_y_);
  
}

int PathPlan::getPathMapValue(int x, int y)
{
  if(x<0 || x>=GRID_SIZE || y<0 || y>=GRID_SIZE){
    ROS_ERROR("get_path_map_value get an illegal input x:%d, y%d", x, y);
    return 1000;
  }
  if(!path_map_initialized_){
    ROS_ERROR("path_map hasn't initialized.");
    return 1000;
  }
  
  return path_map_.at(x, y);
  
}

/*
aStar contrbutor: Chia Xiang Rong
including the predefined set compare part
Waiting to be tested
*/
/*
void PathPlan::aStar()
{
  // make sure the goal cell has been put in the path_map
  initializePathMap();
  
  mat closed_list(GRID_SIZE, GRID_SIZE, fill::zeros);
  mat in_open_list(GRID_SIZE, GRID_SIZE, fill::zeros);
  
  int pos_x_int = (int)floor(pos_x_);
  int pos_y_int = (int)floor(pos_y_);
  int adj_f, adj_g, adj_h;
  
  int x_queue, y_queue; //tmp coord in queue

  std::multiset<node, compareNode> open_list;
  // std::vector<node> open_list;
  node goal = {GOAL_X, GOAL_Y, 0, 0, 0}, min_node, curr_node = {pos_x_int, pos_y_int, -1, -1, -1};
  open_list.insert(goal);
  in_open_list(GOAL_X, GOAL_Y) = 1;
  
  while(!open_list.empty()){
    // find node with min f in queue
    auto it = open_list.begin();
    min_node = *it;
    // std::cout << min_node.x << endl;

    // remove node from open list
    in_open_list(min_node.x, min_node.y) = 0;

    // add to closed list 
    closed_list(min_node.x, min_node.y) = 1; 
    
    // explore adjacent cells to this node
    for(int direction = NORTH; direction <= WEST; direction++){

      // check if there are walls in that direction,
      if(!hasWall(min_node.x, min_node.y, direction)){

        // check if within bounds
        if(direction == NORTH && min_node.y+1 < GRID_SIZE){
          x_queue = min_node.x; y_queue = min_node.y+1;
        }
        if(direction == EAST && min_node.x+1 < GRID_SIZE){
          x_queue = min_node.x+1; y_queue = min_node.y;
        }
        if(direction == SOUTH && min_node.y-1 >= 0){
          x_queue = min_node.x; y_queue = min_node.y-1;
        }
        if(direction == WEST && min_node.x-1 >= 0){
          x_queue = min_node.x-1; y_queue = min_node.y;
        }

        // ignore if in closed list
        if(!closed_list(x_queue, y_queue)){
          // create adj node
          node adj_node;
          adj_node.x = x_queue;
          adj_node.y = y_queue;
          adj_g = min_node.g + 1;
          adj_h = manhattenDist(curr_node, adj_node);
          adj_f = adj_g + adj_h;
          adj_node.f = adj_f;
          adj_node.g = adj_g;
          adj_node.h = adj_h;
          //ROS_INFO("x_queue: %d, y_queue: %d", x_queue, y_queue);

          // not in open list
          if(!in_open_list(x_queue, y_queue)){
            // add to open list
            // ROS_INFO("Open list size: %d, adj_f: %d", open_list.size(), adj_node.f);
            open_list.insert(adj_node);
            in_open_list(x_queue, y_queue) = 1;
          }

          // update path_map
          path_map_(x_queue, y_queue) = min(path_map_(x_queue, y_queue), adj_f);
        }
      }
    }
    open_list.erase(it);
  }
}
*/















