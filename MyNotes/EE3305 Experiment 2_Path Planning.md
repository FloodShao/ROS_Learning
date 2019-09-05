# EE3305 Experiment 2： Path Planning

## Problem
A robot is put in a maze. The robot doesn't know the map of the maze. Given the target coordinate, move the robot to the target while avoid hitting the wall.

## Structure
### 1. Environment
Given a robot simulator provides LaserScan measurement and Odometry measurement. 
Given the simulated maze environment. The map is divided into multiple cells, each cell has four direction where the wall may appears.

### 2. Perception Node
Use the laser scan to detect the wall within the current cell. Pass the detected range to decision making node.

### 3. Decision Node
Maintain and update the map of the maze using the wall detection from Perception Node and localization from Odometry. 
Using Dijkstra Algorithm to find the path with minimum length to the target. 
Decide the next cell the robot needs to heading. Pass the decision to Control Node.

### 4. Control Node
Receives next heading cell from the decision. Use PID Control to stabilize the robot movement.

## Steps:
### 1. Environment checking:
#### Installing turtlebot and upgrade gazebo 9
In Melodic, cmd: `sudo apt-get install ros-kinetic-turtlebot-*`
```
Trouble Shooting
1. There are a lot of pkgs showing error "can not find resources"
[Ans] <https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/>
This is because we are using the old public key to fetching the updated resource. We need to update the public key.

2. Gazebo crashing when launching the navigation node
[Ans] <https://stackoverflow.com/questions/55181205/gazebo-crashing-when-subscribing-to-scan>
This should be gazebo version problem. Update the gazebo 7 should be good.
Just update gazebo using `sudo apt-get upgrade gazebo9`

3. [Error]E: Could not get lock /var/lib/dpkg/lock-frontend - open (11: Resource temporarily unavailable)
E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), is another process using it?
[Solution] 
sudo killall apt apt-get
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock
sudo rm /var/lib/dpkg/lock*
sudo dpkg --configure -a
sudo apt update

4. [error] libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
[Solution] You need to change ~/.ignition/fuel/config.yaml as following.
    url: https://api.ignitionfuel.org
to
    url: https://api.ignitionrobotics.org
```

### 2. Launching the environment
```
cd ee3305_path_plan_v2/bin
./melodic_project_init_world_1.sh
```
And wait until the gazebo shows the maze and the robot

### 3. Launching all the nodes
```
roslaunch ee3305_path_plan_v2 start_all_nodes.launch
```




