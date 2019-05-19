#ifndef VEHICLENULLVEL_H
#define VEHICLENULLVEL_H

#include "task.h"

/**
 * @brief The VehicleNullVel class. It is a non reactive task which is need
 * in manipulation phases where we want the vehicle to stay still while the
 * arm is manipulating something. In such situation, this task should be putted
 * as the highest priority task
 */
class VehicleNullVelTask : public Task
{
public:
  VehicleNullVelTask(int dimension, bool eqType, std::string robotName);

  int updateMatrices(struct Infos* const robInfo);

private:
  int setActivation();
  int setJacobian(Eigen::Matrix4d wTv_eigen);
  int setReference();

};

#endif // VEHICLENULLVEL_H
