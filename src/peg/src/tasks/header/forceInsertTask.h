#ifndef FORCEINSERTTASK_H
#define FORCEINSERTTASK_H

#include "task.h"
#include "../../support/header/conversions.h"
#include "../../support/header/formulas.h"

/**
 * @brief The ForceInsertTask class to help the inserting phase with force torque sensor
 * this task is to generate velocity such that the forces are nullified
 *
 * DIMENSION: this parameters tells the type of the task:
 *   1 Norm of only forces (linear one) is the element to nullify
 *   2 Norm of both forces and torques are the two element to nullify BEST ONE
 *   3 Forces (linear) all its three components
 *   6 NOT IMPLEMENTED all components of forces and torques
 *
 *   Without norm task impose useless constrain to lower priority task. So the
 *   best is use the norms
 */
class ForceInsertTask : public Task
{
public:
  ForceInsertTask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);
private:
  void setActivation(Eigen::Vector3d force, Eigen::Vector3d torque);
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot,
                   Eigen::Vector3d force,  Eigen::Vector3d torque);
  void setReference(Eigen::Vector3d force, Eigen::Vector3d torque);
};

#endif // FORCEINSERTTASK_H
