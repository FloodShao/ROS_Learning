# Bugs fixing (2019-10-30)
1. Solving the problem of wrong wall detection due to the width of the wall in function checkWall()
2. Solving the problem that some of the cells may not be reached in dijkstra due to the error detection of the wall.
3. Solving the matrix overflow error when checking the visited_map and is_queued
4. Optimize the pid control parameters
5. Optimize the Wall detection threshold
6. Addressing the condition of setting the wall in adjacent cell in setWall()
Thanks Chia Xiang Rong for reporting some of the bugs.
7. Changing the launching world bash file. Avoid copying the world file in the system ROS package folder.


# Bugs fixing (2019-10-24)
1. Optimize the pid control parameters
2. Fix the bugs of path plan node, where the path_map only initialize at the very first stage, but in the following stage, the path_map get wrong distance.
3. Fix the box_init and bot_goal in the world file, which have the height of 0.1m, and could get the robot stuck at the edge.

# Instruction
(2019-10-24 Version)
To run this program:
1. clone the original code to your catkin workspace
2. run the bash file to install the dependencies ./bin/install_turtlebot3.sh (remember to change the ros version)
2. catkin build this package
3. run the bash file in ./bin/launch_world.sh
4. wait gazebo to start the environment
5. launch ./launch/start_all_nodes.launch


(No longer valid)
To run this program:
1. clone the original code to your catkin workspace
2. catkin build this package
3. run the bash file in ./bin/kinetic_project_init_world_1.sh
4. wait gazebo to start the environment
5. launch ./launch/start_all_nodes.launch





