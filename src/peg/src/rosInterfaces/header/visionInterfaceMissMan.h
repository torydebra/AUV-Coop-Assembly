#ifndef VISIONINTERFACEMISSMAN_H
#define VISIONINTERFACEMISSMAN_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Core>

#include "../../support/header/conversions.h"

class VisionInterfaceMissMan
{
public:
  VisionInterfaceMissMan(ros::NodeHandle nh, std::string robotName);
  int getHoleTransform(Eigen::Matrix4d *wThole);
private:
  std::string robotName;
  std::string topicRoot;
  ros::Subscriber subHoleTransform;

  geometry_msgs::Transform wThole_priv;

  void subHoleTransformCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);


};

#endif // VISIONINTERFACEMISSMAN_H
