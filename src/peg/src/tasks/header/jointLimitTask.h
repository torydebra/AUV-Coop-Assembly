#ifndef JOINTLIMITTASK_H
#define JOINTLIMITTASK_H

#include "task.h"

class JointLimitTask : public Task
{
public:
  JointLimitTask(int dimension, bool eqType, std::string robotName);
  ~JointLimitTask();
  int updateMatrices(struct Infos* const robInfo);
private:
  int setActivation(std::vector<double> jState);
  int setJacobian();
  int setReference(std::vector<double> jState);

  double* safeGuardUp;
  double* safeGuardLow;
  double* halfPoint;
};

#endif // JOINTLIMITTASK_H
