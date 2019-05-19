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
  CoordInterfaceCoord(ros::NodeHandle nh, std::string robotNameA,
                      std::string robotNameB);
  bool getReadyBothRob();

  int getMatricesFromRobs(Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVelA,
                          Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVelB,
                          Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelToolA,
                          Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelToolB);

  void publishCoopVel(Eigen::Matrix<double, VEHICLE_DOF, 1> coopVel);
  void publishStartBoth(bool start);


private:
  bool getReadyRobA();
  bool getReadyRobB();
  int getNonCoopCartVel(Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVel_eigen,
                        std::string robotName);
  int getJJsharp(Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelTool_eigen,
                 std::string robotName);


  std::string robotNameA;
  std::string robotNameB;
  bool readyRobA;
  bool readyRobB;
  ros::Subscriber readyRobASub;
  ros::Subscriber readyRobBSub;
  ros::Subscriber subCoordFromMMA;
  ros::Subscriber subCoordFromMMB;

  ros::Publisher startBothPub;
  ros::Publisher coopVelPub;


  std::vector<double> tempXdotA;
  std::vector<double> tempJJsharpA;
  std::vector<double> tempXdotB;
  std::vector<double> tempJJsharpB;

  bool readxDotA;
  bool readxDotB;
  bool readJJsharpA;
  bool readJJsharpB;



  void readyRobASubCallback(const std_msgs::Bool::ConstPtr& start);
  void subCoordFromMMACallBack(const peg_msgs::toCoord::ConstPtr& msg);
  void readyRobBSubCallback(const std_msgs::Bool::ConstPtr& start);
  void subCoordFromMMBCallBack(const peg_msgs::toCoord::ConstPtr& msg);


};

#endif // COORDINTERFACECOORD_H
