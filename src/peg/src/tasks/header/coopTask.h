#ifndef COOPTASK_H
#define COOPTASK_H

#include "task.h"
#include "../../support/header/conversions.h"

/**
 * @brief The CoopTask class
 */
class CoopTask : public Task
{
public:
  CoopTask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);
private:
  void setActivation();
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_Jtool_robot);
  void setReference(Eigen::Matrix<double, VEHICLE_DOF, 1> coopCartVel);
};

#endif // COOPTASK_H
