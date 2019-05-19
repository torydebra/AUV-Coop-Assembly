#ifndef OBSTACLEAVOIDVEHICLETASK_H
#define OBSTACLEAVOIDVEHICLETASK_H

#include "task.h"
#include "../../support/header/conversions.h"

/**
 * @brief The ObstacleAvoidVehicleTask class
 * (where the obstacle is the other vehicle, at the moment only for this)
 */
class ObstacleAvoidVehicleTask : public Task
{
public:
  ObstacleAvoidVehicleTask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);

private:
  void setActivation(double w_distNorm);
  void setJacobian(Eigen::Matrix4d wTv, Eigen::Vector3d w_dist_normal);
  void setReference(double w_distNorm);
  double safe_dist; //safe distance added to make the robot not touch
};

#endif // OBSTACLEAVOIDVEHICLETASK_H
