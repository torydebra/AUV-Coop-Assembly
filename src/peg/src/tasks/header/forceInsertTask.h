#ifndef FORCEINSERTTASK_H
#define FORCEINSERTTASK_H

#include "task.h"
#include "../../support/header/conversions.h"
#include "../../support/header/formulas.h"


class ForceInsertTask : public Task
{
public:
  ForceInsertTask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);
private:
  void setActivation(Eigen::Vector3d force, Eigen::Vector3d torque);
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot);
  void setReference(Eigen::Vector3d force, Eigen::Vector3d torque);
};

#endif // FORCEINSERTTASK_H
