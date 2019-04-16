#ifndef WORLDINTERFACE_H
#define WORLDINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

#include "../../support/header/conversions.h"


class WorldInterface
{
public:
  WorldInterface();
  WorldInterface(std::string callerName, std::string toolName);
  int init();
  int getwTt(Eigen::Matrix4d* wTt_eigen);

private:
  std::string callerName;
  std::string toolName;
  tf::TransformListener tfListener;

};

#endif // WORLDINTERFACE_H
