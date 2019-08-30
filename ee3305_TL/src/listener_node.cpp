#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  //Subscribe the topic 'chatter', with buffer size 1000, Whennever I receive a new message, a function called 'chatterCallback' will be trigged
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  //Alwayes listening to the topic to see if there is new message come in
  ros::spin();

  return 0;
}