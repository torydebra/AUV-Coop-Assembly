#ifndef MANIPULABILITYTASK_H
#define MANIPULABILITYTASK_H

#include <Eigen/Core>
#include "task.h"

class ManipulabilityTask : public Task
{
public:
  ManipulabilityTask(int dimension, int dof, bool eqType);
  //~ManipulabilityTask();
  int updateMatrices(struct Transforms* const transf);
private:
  void setActivation();
  int setJacobian();
  void setReference();
  double mu; //manipulability value
};

#endif // MANIPULABILITYTASK_H
