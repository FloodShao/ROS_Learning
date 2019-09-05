sudo cp ../worlds/test_world_1.world /opt/ros/melodic/share/turtlebot3_gazebo/worlds/
sudo cp ../launch/turtlebot3_maze.launch /opt/ros/melodic/share/turtlebot3_gazebo/launch/
export TURTLEBOT3_MODEL="burger"
export INIT_X="0.5"
export INIT_Y="0.5"
export INIT_YAW="1.57"
roslaunch turtlebot3_gazebo turtlebot3_maze.launch

