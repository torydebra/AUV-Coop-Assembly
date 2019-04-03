#ifndef VEHICLECONSTRAINVELTASK_H
#define VEHICLECONSTRAINVELTASK_H

#include "task.h"
#include "../../support/header/conversions.h"

/**
 * @brief The vehicleConstrainVelTask class
 * This task is a non-reactive one used to to
 * constrain the velocities of the vehicle to the actual ones.
 * It is needed for the parallel arm-vehicle coordination scheme,
 * And it will be the highest priority task (being a constraint one)
 * of the second list of tasks
 */
class VehicleConstrainVelTask : public Task
{
public:
  VehicleConstrainVelTask(int dimension, bool eqType);
  int updateMatrices(struct Infos* const robInfo);

private:
  void setActivation();
  void setJacobian();
  void setReference(std::vector<double> actualVel);

};

#endif // VEHICLECONSTRAINVELTASK_H
