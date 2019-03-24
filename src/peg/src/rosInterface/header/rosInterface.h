#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include "../../support/header/conversions.h"
#include "../../support/header/defines.h"

/** @brief RosInterface: a ros node responsible of taken info from simulator and robot sensors,
 * and of given commands back. It is the intermiate layer between robot and mission manager ("main")
**/
class RosInterface
{
public:
  RosInterface(std::string robotName, std::string topicTwist, int argc, char **argv);
  int init();
  int getwTv(Eigen::Matrix4d* wTv_eigen);
  int sendQDot(std::vector<double> qDot);


private:
  std::string robotName;
  ros::Publisher pubTwist;
  std::string topicTwist;
  tf::TransformListener tfListener_wTv;
  tf::StampedTransform wTv_tf;
};

#endif // ROSINTERFACE_H
