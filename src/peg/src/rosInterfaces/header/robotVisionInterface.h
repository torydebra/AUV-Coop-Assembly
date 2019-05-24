#ifndef ROBOTVISIONINTERFACE_H
#define ROBOTVISIONINTERFACE_H

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>


#include <Eigen/Core>
#include "../../support/header/conversions.h"
//#include "../../support/header/defines.h"

/** @brief robotInterface: a ros node responsible of taken info from simulator and robot sensors,
 * and of given commands back. It is the intermiate layer between robot and mission manager ("main")
 * This "Vision" Version is specialized for a robot with no arm and horizontal cameras used to detect hole
 * and help the insertion phase.
 * @note from ROS doc: "image_transport should always be used to publish and subscribe to images"
 * (instead of simple ros::subscriber)
**/
class RobotVisionInterface
{
public:
  RobotVisionInterface(ros::NodeHandle nh, std::string robotName);
  int init();
  int getwTv(Eigen::Matrix4d* wTv_eigen);
  int sendyDot(std::vector<double> yDot);


  int getLeftImage(cv::Mat *imageCV);
  int getRightImage(cv::Mat *imageCV);
  int getRangeRightImage(cv::Mat *imageCV);
  void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg);
  void imageRightCallback(const sensor_msgs::ImageConstPtr& msg);
  void imageRangeRightCallback(const sensor_msgs::ImageConstPtr& msg);


private:
  std::string robotName; //for tf listener (girona500_A,B)
  std::string topicRoot; //for publishing yDot and subscribing to sensors ("/uwsim/g500_A/")
  tf::TransformListener tfListener;
  ros::Publisher pubTwist;

  image_transport::Subscriber subLeftImage;
  image_transport::Subscriber subRightImage;
  image_transport::Subscriber subRangeRightImage;

  sensor_msgs::ImageConstPtr leftImage;
  sensor_msgs::ImageConstPtr rightImage;
  sensor_msgs::ImageConstPtr rangeRightImage;




};

#endif // ROBOTVISIONINTERFACE_H
