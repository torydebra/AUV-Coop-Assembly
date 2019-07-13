#ifndef PIPEREACHTASK_H
#define PIPEREACHTASK_H

#include "../../support/header/conversions.h"
#include "../../support/header/formulas.h"
#include "task.h"

/**
 * @brief The VehArmType enum enum to set different kind of this task:
 *   ONLYVEH: the reach of goal for the pipe is accomplished by moving only the vehicle
 *    which means that in the jacobian the arm part (first 4 columns) is all zero
 *    note that this means that this task generate zero velocities for the arm, but other task
 *    may generate some velocities.
 *  ONLYARM: similar as above. Useful for fine position eg during the insertion phase.
 *  BOTH: no zero are constrainted in the jacobian
 * @note @todo maybe use another task to constrain the velocities of vehicle (it works probably)
 *  and the arm (tried fast and it does not work, the two pegs go away a lot)
 */
enum VehArmType {ONLYVEH, ONLYARM, BOTH};


class PipeReachTask : public Task {

public:
  PipeReachTask(int dimension, bool eqType, std::string robotName, VehArmType vehArmType);
  int updateMatrices(struct Infos* const robInfo);

private:
  int setActivation();
  void setReference(Eigen::Matrix4d wTgoaltool_eigen, Eigen::Matrix4d wTtool_eigen);
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot);

  double gainLin;
  double gainAng;

  VehArmType vehArmType;
};
#endif // PIPEREACHTASK_H
