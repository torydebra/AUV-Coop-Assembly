#ifndef VISIONINTERFACECOORD_H
#define VISIONINTERFACECOORD_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Core>

#include "../../support/header/conversions.h"

/**
 * @brief The VisionInterfaceCoord class. It is the coordinator interface which take the
 * hole estimated pose from the Vision Robot
 */
class VisionInterfaceCoord
{
public:
  VisionInterfaceCoord(ros::NodeHandle nh, std::string nodeName);
  int getHoleTransform(Eigen::Matrix4d *wThole);
private:
  std::string nodeName;
  std::string topicRoot;
  ros::Subscriber subHoleTransform;

  geometry_msgs::Transform wThole_priv;
  bool arrived;

  void subHoleTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);


};

#endif // VISIONINTERFACECOORD_H
