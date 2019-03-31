#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include "../../support/header/conversions.h"
#include "../../support/header/defines.h"

/** @brief RosInterface: a ros node responsible of taken info from simulator and robot sensors,
 * and of given commands back. It is the intermiate layer between robot and mission manager ("main")
**/
class RosInterface
{
public:
  RosInterface(std::string robotName, std::string topicRoot, int argc, char **argv);
  int init();
  int getwTv(Eigen::Matrix4d* wTv_eigen);
//  int getvTee(Eigen::Matrix4d* vTee_eigen);
//  int getvTjoints(std::vector<Eigen::Matrix4d>* vTjoints);
  int getJointState(std::vector<double>* jState);
  int sendQDot(std::vector<double> qDot);
  void spinOnce();


private:
  std::string robotName; //for tf listener (girona500_A,B)
  std::string topicRoot; //for publishing qDot and subscribing to sensors ("/uwsim/g500_A/")
  tf::TransformListener tfListener_wTv;
  ros::Publisher pubTwist;
  ros::Publisher pubJoint;
  ros::Subscriber subJointState;
  //std::string topicJoint;

  std::vector<double> jState_priv;
  void subJointStateCallback(const sensor_msgs::JointState& js);
};

#endif // ROSINTERFACE_H
