#ifndef OBSTACLEAVOIDEETASK_H
#define OBSTACLEAVOIDEETASK_H

#include "task.h"
#include "../../support/header/conversions.h"

/**
 * @brief The ObstacleAvoidEETask class
 * Usage of this task can be for avoiding with the end effector, ground, objects, or other robot
 * TODO for each obstacle implement a task OTHERWISE set this task not as scalar one but one row
 * for each object to avoid.
 * at the moment it avoid only the seafloor
 */
class ObstacleAvoidEETask : public Task
{
public:
  ObstacleAvoidEETask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);

private:
  void setActivation(double w_dist);
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot, double w_dist);
  void setReference(double w_dist);
  double safe_dist; //safe distance from the obstacle
};

#endif // OBSTACLEAVOIDEETASK_H
