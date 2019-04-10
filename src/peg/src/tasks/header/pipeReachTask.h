#ifndef PIPEREACHTASK_H
#define PIPEREACHTASK_H

#include "../../support/header/conversions.h"
#include "../../support/header/formulas.h"
#include "task.h"

class PipeReachTask : public Task {

public:
  PipeReachTask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);

private:
  int setActivation();
  void setReference(Eigen::Matrix4d wTgoaltool_eigen, Eigen::Matrix4d wTtool_eigen);
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot);
};
#endif // PIPEREACHTASK_H
