# Bugs fixing (2019-10-24)
1. Optimize the pid control parameters
2. Fix the bugs of path plan node, where the path_map only initialize at the very first stage, but in the following stage, the path_map get wrong distance.
3. Fix the box_init and bot_goal in the world file, which have the height of 0.1m, and could get the robot stuck at the edge.

# Instruction
To run this program:
1. clone the original code to your catkin workspace
2. catkin build this package
3. run the bash file in ./bin/kinetic_project_init_world_1.sh
4. wait gazebo to start the environment
5. launch ./launch/start_all_nodes.launch





