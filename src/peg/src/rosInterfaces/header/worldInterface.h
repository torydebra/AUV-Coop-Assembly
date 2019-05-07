#ifndef WORLDINTERFACE_H
#define WORLDINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

#include "../../support/header/conversions.h"

/**
 * @brief The WorldInterface class
 * to get position from world to anything else.
 * @note waitReady should be called to wait/check if the two frames exist in ros
 */
class WorldInterface
{
public:
  /**
   * @brief WorldInterface
   * @param callerName
   */
  WorldInterface(std::string callerName);
  int waitReady(std::string objName);
  int waitReady(std::string firstFrame, std::string secondFrame);


  int getwT(Eigen::Matrix4d *wTobj, std::string objName);
  int getT(Eigen::Matrix4d* xTx_eigen, std::string firstFrame, std::string secondFrame);
  int getwPos(Eigen::Vector3d* pos, std::string robotName);



private:
  std::string callerName;
  tf::TransformListener tfListener;

};

#endif // WORLDINTERFACE_H
