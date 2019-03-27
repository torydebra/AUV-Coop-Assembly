#ifndef ENDEFFECTORREACHTASK_H
#define ENDEFFECTORREACHTASK_H
#include "task.h" // task.h will include other header (like cmat and eigen)

/** @brief Task to make the end effector reach a goal (both linear and angular position)
 * @note this task makes calculations with things projected on vehicle frame
*/
class EndEffectorReachTask : public Task {

public:
  EndEffectorReachTask(int dimension, int dof, bool eqType);
  int updateMatrices(struct Transforms* const transf);
private:
  int setActivation();
  int setJacobian(std::vector<Eigen::Matrix4d> vTjoints, Eigen::Matrix4d vTee_eigen);
  int setReference(
      Eigen::Matrix4d vTee_eigen, Eigen::Matrix4d vTgoalEE_eigen, Eigen::Matrix4d wTv_eigen);
};




#endif // ENDEFFECTORREACHTASK_H
