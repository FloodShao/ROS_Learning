#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  //You node is activated by this init function, "talker" is your node_name
  ros::init(argc, argv, "talker");

  //Create a nodehandle, all the operations are through the node handle
  ros::NodeHandle n;

  //Define the publisher topic, topic name "chatter", msgs type "std_msgs/String", publish buffer size 1000
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  //Define the frequency (Hz) to wake up your node and do something
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    //Log, this is hard code in the screen, not the real message
    ROS_INFO("%s", msg.data.c_str());

    //Here, the message is actually sent out
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}