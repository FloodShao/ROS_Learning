#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Floar32MultiArray.h>


class RangeDetect{

private:
  ros::Subscriber scan_sub_;
  ros::Publisher range_detect_pub_;
  
public:

};
