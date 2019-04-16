#ifndef WORLDINTERFACE_H
#define WORLDINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

#include "../../support/header/conversions.h"


class WorldInterface
{
public:
  /**
   * @brief WorldInterface
   * @param callerName
   * @param toolName
   * @param toolName2 actually, the pipe2 attached to robotB. Useful
   * know its position to see how distant is from pipe1 to understand the
   * mechanical stress that we would apply to a real pipe in real world
   * Actually used by logger for this scope
   */
  WorldInterface(std::string callerName, std::string toolName,
                 std::string toolName2 = "");
  int init();
  int getwTt(Eigen::Matrix4d* wTt_eigen, bool tool2 = false);
  int getRobPos(Eigen::Vector3d* pos, std::string robotName);

private:
  std::string toolName;
  std::string toolName2;
  std::string callerName;
  tf::TransformListener tfListener;

};

#endif // WORLDINTERFACE_H
