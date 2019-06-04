#ifndef ARMNULLVELTASK_H
#define ARMNULLVELTASK_H

#include "task.h"

/**
 * @brief The ArmNullVelTask class. It is a non reactive task which is needed
 * when vehicle moves, because we want big movements to be made only by vehicle.
 * In such situation, this task should be putted as the highest priority task
 */
class ArmNullVelTask : public Task
{
public:
  ArmNullVelTask(int dimension, bool eqType, std::string robotName);

  int updateMatrices(struct Infos* const robInfo);

private:
  int setActivation();
  int setJacobian();
  int setReference();

};

#endif // ARMNULLVELTASK_H
