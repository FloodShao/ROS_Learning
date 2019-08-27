#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


namespace enc = sensor_msgs::image_encodings;

static const char window_rgb[] = "RGB Camera";
static const char window_dep[] = "Depth Camera";
static const char window_blur[] = "Blur Image";
static const char window_cdep[] = "Colored Depth Image";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_rgb_sub_;
  image_transport::Subscriber img_dep_sub_;

public:
  ImageConverter(): it_(nh_)
  {
    img_rgb_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::image_rgb, this);
    cv::namedWindow(window_rgb);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(window_rgb);
  }

  void image_rgb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8); // encoding
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow(window_rgb, cv_ptr->image); // using pointer to display
    cv::waitKey(3);
  }

  void image_dep(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat blur_img;
    try{
      cv_ptr = cv_bridge::toCvShare(msg, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    double minVal, maxVal;
    minMaxLoc(cv_ptr->image,&minVal,&maxVal);
    cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    /* To show a color map of depth image*/
    minMaxIdx(cv_ptr->image, &minVal, &maxVal);
    cv::Mat adjMap, falseColorMap;
    cv_ptr->image.convertTo(adjMap, CV_8UC1, 255.0/(maxVal - minVal), -minVal);
    applyColorMap(adjMap, falseColorMap, cv::COLORMAP_AUTUMN);

    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_operations");
  ImageConverter ic;
  ros::spin();
  return 0;
}
