sudo cp ../worlds/test_world_1.world /opt/ros/melodic/share/turtlebot3_gazebo/worlds/
export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/melodic/share/turtlebot3_gazebo/worlds/test_world_1.world"
export ROBOT_INITIAL_POSE="-x 0.5 -y 0.5 -Y 1.57"
export TURTLEBOT3_MODEL="burger"
roslaunch turtlebot3_gazebo turtlebot3_world.launch
