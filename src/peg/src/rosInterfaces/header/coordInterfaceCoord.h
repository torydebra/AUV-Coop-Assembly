#ifndef COORDINTERFACECOORD_H
#define COORDINTERFACECOORD_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <peg/toCoord_msg.h>
#include <Eigen/Core>
#include "../../support/header/defines.h"

/**
 * @brief The CoordInterfaceCoord class
 * @warning the SpinOnce is made by caller of this class
 */
class CoordInterfaceCoord
{
public:
  CoordInterfaceCoord(ros::NodeHandle nh, std::string robotName);
  int getxDot(Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVel_eigen);
  int getJJsharp(Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelTool_eigen);

private:
  std::string robotName;
  ros::Subscriber subCoordFromMM;

  std::vector<double> tempXdot;
  std::vector<double> tempJJsharp;

  void subCoordFromMMCallBack(const peg::toCoord_msg::ConstPtr& msg);


};

#endif // COORDINTERFACECOORD_H
