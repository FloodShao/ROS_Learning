cp ../worlds/test_world_1.world /opt/ros/kinetic/share/husky_gazebo/worlds/
export HUSKY_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/husky_gazebo/worlds/test_world_1.world"
export HUSKY_INIT_X="0.6"
export HUSKY_INIT_Y="0.5"
export HUSKY_INIT_Z="0"
export HUSKY_INIT_YAW="1.57"
roslaunch husky_gazebo husky_empty_world_mylaunch.launch