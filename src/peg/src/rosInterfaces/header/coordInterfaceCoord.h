#ifndef COORDINTERFACECOORD_H
#define COORDINTERFACECOORD_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <peg_msgs/toCoord.h>
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
  bool getReadyRob();
  int getNonCoopCartVel(Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVel_eigen);
  int getJJsharp(Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelTool_eigen);

private:
  std::string robotName;
  bool readyRob;
  ros::Subscriber readyRobSub;
  ros::Subscriber subCoordFromMM;

  std::vector<double> tempXdot;
  std::vector<double> tempJJsharp;

  void readyRobSubCallback(const std_msgs::Bool::ConstPtr& start);
  void subCoordFromMMCallBack(const peg_msgs::toCoord::ConstPtr& msg);


};

#endif // COORDINTERFACECOORD_H
