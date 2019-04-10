#ifndef ARMSHAPETASK_H
#define ARMSHAPETASK_H

#include "task.h"

/**
 * @brief The ArmShapeTask class
 * Preferred arm Shape: an optimization task. It is a non scalar task, each
 * joint is controlled separately and bringed to its preferred position.
 * The activation is always the identity, so it is important that these task will be the
 * last one which modifify qdot for the arm.
 *
 * Another method could be set an activation different from identity e/o
 * using the norm (in this case this task become scalar)
 * VERY important: if task scalar and norm used, the activation is fundamental to not have
 * a norm which go towards zero (thus generating infinite velocities)
 */
class ArmShapeTask : public Task
{
public:
  ArmShapeTask(int dimension, bool eqType, std::string robotName);
  int updateMatrices(struct Infos* const robInfo);
private:
  int setActivation();
  int setJacobian();
  int setReference(std::vector<double> jointGoal, std::vector<double> jState);
};

#endif // ARMSHAPETASK_H
