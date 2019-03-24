#ifndef VEHICLEREACHTASK_H
#define VEHICLEREACHTASK_H

#include "task.h" // task.h will include other header (like cmat and eigen)

/** @brief Task to make the vehicle reach a goal (both linear and angular position)
*/
class VehicleReachTask : public Task {

public:
  VehicleReachTask(int dimension, int dof, bool eqType);
  VehicleReachTask(int dimension, bool eqType);

  int updateMatrices(struct Transforms* const transf);

private:
  int setActivation();
  int setJacobian(Eigen::Matrix4d wTv_eigen);
  int setReference(
      Eigen::Matrix4d wTv_eigen, Eigen::Matrix4d wTg_eigen);

};

#endif // VEHICLEREACHTASK_H
