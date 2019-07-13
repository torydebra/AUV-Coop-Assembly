#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <boost/circular_buffer.hpp>

#include <Eigen/Core>
#include "../../support/header/conversions.h"
#include "../../support/header/defines.h"
#include "../../support/header/formulas.h"


#define NELEMENTQUEUE 10 //number of element in queue for force and torque

/** @brief robotInterface: a ros node responsible of taken info from simulator and robot sensors,
 * and of given commands back. It is the intermiate layer between actual robot and mission manager ("main")
 *
 * @note about the force/torque sensor:
 *   NOT VALID ANYMORE WITH RESOLVED LAG ISSUE WITH SIMULATION? BUT MAYBE USEFUL TO HAVE A SMOOTH BEHAVIOUR **********************************************
 *   we have to make a mean: the sensor on uwsim give values but often they are intervalled with
 *   values = 0. this cause chattering and the robot dont move at all because 0-x velocity are intermittely
 *   given. So idea is use a queue and make a mean of the last 10 15 20 values of the sensor, and give the mean
 *   when caller calls getForceTorque.
 *   This problem can be given by simulator, by the sensor, or even by the physics engine which has difficulty
 *   in calculating collision between cilinder and inner hole.
 *   This is done with two vectors (one for force, one for torque) of circular buffers.
 *   Each vector contain 3 fixed size circular buffer of double, one for each direction (x y z)
 *   in this way we can use std::accumulate to sum all element of each buffer to the divided and
 *   make the mean
 *   NOT VALID ANYMORE WITH RESOLVED LAG ISSUE WITH SIMULATION? *************************************************
 *
**/
class RobotInterface
{
public:
  RobotInterface(ros::NodeHandle nh, std::string robotName);
  int init();
  int getwTv(Eigen::Matrix4d* wTv_eigen);
  int getJointState(std::vector<double>* jState);
  int sendyDot(std::vector<double> yDot);

  int getForceTorque(Eigen::Vector3d* force, Eigen::Vector3d* torque);

  //int getwTjoints(std::vector<Eigen::Matrix4d> *wTjoints);


private:
  std::string robotName; //for tf listener (girona500_A,B)
  std::string topicRoot; //for publishing yDot and subscribing to sensors ("/uwsim/g500_A/")
  tf::TransformListener tfListener;
  ros::Publisher pubTwist;
  ros::Publisher pubJoint;
  ros::Subscriber subJointState;
  ros::Subscriber subForceTorque;

  std::vector<double> jState_priv;
  //std::vector<double> force_priv; with queues they dont need to be member of class
  //std::vector<double> torque_priv;
  void subJointStateCallback(const sensor_msgs::JointState& js);
  void subForceTorqueCallback(const geometry_msgs::WrenchStamped& msg);

  std::vector<boost::circular_buffer<double>> vectorForceQueue;
  std::vector<boost::circular_buffer<double>> vectorTorqueQueue;



};


#endif // ROBOTINTERFACE_H
