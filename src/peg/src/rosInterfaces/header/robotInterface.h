#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include "../../support/header/conversions.h"
#include "../../support/header/defines.h"

/** @brief robotInterface: a ros node responsible of taken info from simulator and robot sensors,
 * and of given commands back. It is the intermiate layer between actual robot and mission manager ("main")
**/
class RobotInterface
{
public:
  RobotInterface(ros::NodeHandle nh, std::string robotName);
  int init();
  int getwTv(Eigen::Matrix4d* wTv_eigen);
  int getJointState(std::vector<double>* jState);
  int sendyDot(std::vector<double> yDot);

  //int getwTjoints(std::vector<Eigen::Matrix4d> *wTjoints);


private:
  std::string robotName; //for tf listener (girona500_A,B)
  std::string topicRoot; //for publishing yDot and subscribing to sensors ("/uwsim/g500_A/")
  tf::TransformListener tfListener;
  ros::Publisher pubTwist;
  ros::Publisher pubJoint;
  ros::Subscriber subJointState;

  std::vector<double> jState_priv;
  void subJointStateCallback(const sensor_msgs::JointState& js);
};

#endif // ROBOTINTERFACE_H
