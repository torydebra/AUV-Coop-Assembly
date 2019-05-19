#ifndef VISIONINTERFACEVISION_H
#define VISIONINTERFACEVISION_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>

#include "../../support/header/conversions.h"



class VisionInterfaceVision
{
public:
  VisionInterfaceVision(ros::NodeHandle nh, std::string robotName);
  int publishHoleTransform(Eigen::Matrix4d wThole);


private:
  std::string robotName;
  std::string topicRoot;
  ros::Publisher pubHoleTransform;


};

#endif // VISIONINTERFACEVISION_H
