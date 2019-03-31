#ifndef ENDEFFECTORREACHTASK_H
#define ENDEFFECTORREACHTASK_H

#include "../../support/header/conversions.h"
#include "../../support/header/formulas.h"
#include "task.h"

/** @brief Task to make the end effector reach a goal (both linear and angular position)
 * @note this task makes calculations with things projected on vehicle frame
*/
class EndEffectorReachTask : public Task {

public:
  EndEffectorReachTask(int dimension, int dof, bool eqType);
  int updateMatrices(struct Infos* const robInfo);
private:
  int setActivation();
  int setReference(Eigen::Matrix4d wTgoalEE_eigen, Eigen::Matrix4d wTee_eigen);
  //int setJacobian(std::vector<Eigen::Matrix4d> vTjoints, Eigen::Matrix4d vTee_eigen);
  //int setReference(
      //Eigen::Matrix4d vTee_eigen, Eigen::Matrix4d vTgoalEE_eigen, Eigen::Matrix4d wTv_eigen);
};




#endif // ENDEFFECTORREACHTASK_H
