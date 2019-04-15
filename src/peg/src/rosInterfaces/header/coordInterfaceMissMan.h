#ifndef COORDINTERFACEMISSMAN_H
#define COORDINTERFACEMISSMAN_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <peg_msgs/toCoord.h>
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include "../../support/header/defines.h"

/**
 * @brief The CoordInterface class
 * class to deal with ros comunications (pub/sub)
 * from MissionManager node to Coordinator node
 */
class CoordInterfaceMissMan
{
public:
  CoordInterfaceMissMan(ros::NodeHandle nh, std::string robotName);
  bool getStartFromCoord();
  void pubIamReadyOrNot(bool ready);

  int publishToCoord(Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVel_eigen,
                     Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelTool_eigen);

  int getCoopCartVel(Eigen::Matrix<double, VEHICLE_DOF, 1> *coopCartVel_eigen);

private:
  std::string robotName;

  ros::Publisher readyPub;

  ros::Subscriber startSub;
  bool start;
  void startSubCallback(const std_msgs::Bool::ConstPtr& start);


  ros::Publisher pubToCoord;
  peg_msgs::toCoord toCoordMsg;

  ros::Subscriber coopVelSub;
  std::vector<double> tempCoopVel;
  void subMMFromCoordCallBack(const geometry_msgs::TwistStamped& msg);

};

#endif // COORDINTERFACEMISSMAN_H
