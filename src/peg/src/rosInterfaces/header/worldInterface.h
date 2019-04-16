#ifndef WORLDINTERFACE_H
#define WORLDINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

#include "../../support/header/conversions.h"


class WorldInterface
{
public:
  WorldInterface(std::string callerName, std::string toolName);
  int init();
  int getwTt(Eigen::Matrix4d* wTt_eigen);
  std::string toolName;


private:
  std::string callerName;
  tf::TransformListener tfListener;

};

#endif // WORLDINTERFACE_H
