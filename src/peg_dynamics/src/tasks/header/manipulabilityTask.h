#ifndef MANIPULABILITYTASK_H
#define MANIPULABILITYTASK_H

#include <Eigen/Core>
#include "task.h"

class ManipulabilityTask : public Task
{
public:
  ManipulabilityTask(int dimension, bool eqType, std::string robotName);
  //~ManipulabilityTask();
  int updateMatrices(struct Infos* const robInfo);
private:
  void setActivation();
  int setJacobian();
  void setReference();
  double mu; //manipulability value
};

#endif // MANIPULABILITYTASK_H
