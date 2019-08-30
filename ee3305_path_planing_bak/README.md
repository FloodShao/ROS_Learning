# Path Planing (Under Constructure)

## Environment
* Ubuntu 16.04
* ROS kinetic

## Installing turtlebot 
cmd: `sudo apt-get install ros-kinetic-turtlebot-*`
### Trouble Shooting
1. There are a lot of pkgs showing error "can not find resources"
[Ans] <https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/>
This is because we are using the old public key to fetching the updated resource. We need to update the public key.

2. Gazebo crashing when launching the navigation node
[Ans] <https://stackoverflow.com/questions/55181205/gazebo-crashing-when-subscribing-to-scan>
This should be gazebo version problem. Update the gazebo 7 should be good.


