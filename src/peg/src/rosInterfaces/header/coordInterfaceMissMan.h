#ifndef COORDINTERFACEMISSMAN_H
#define COORDINTERFACEMISSMAN_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <peg/toCoord_msg.h>
#include <Eigen/Core>
#include "../../support/header/defines.h"
#include "../../support/header/formulas.h"
#include "../../support/header/conversions.h"
#include "../../helper/header/infos.h"

/**
 * @brief The CoordInterface class
 * class to deal with ros comunications (pub/sub)
 * from MissionManager node to Coordinator node
 */
class CoordInterfaceMissMan
{
public:
  CoordInterfaceMissMan(ros::NodeHandle nh, std::string robotName);
  int publishToCoord(Infos* robInfo, std::vector<double> yDot);
private:
  std::string robotName;
  ros::Publisher pubToCoord;
  peg::toCoord_msg toCoordMsg;


};

#endif // COORDINTERFACEMISSMAN_H
