#ifndef COLLISIONPROPAGATOR_H
#define COLLISIONPROPAGATOR_H

#include <Eigen/Core>

#include "../../support/header/defines.h"
#include "../../support/header/conversions.h"


/**
 * @brief The CollisionPropagator class
 * It is used to propagate a force suffered by the tip (where the passed jacob world_J_veh_tip and
 * trasf matrix wTpegtip point) to the whole arm, providing the resultant
 * joint velocities. These velocities act as a "disturb" so they must be added to
 * the control vector output of the tpik procedure.
 *
 * Class with only static method, so no costructor is callable. In practice
 * this is used as namespace, but used the class so private method can be here
 */
class CollisionPropagator
{
public:
  static std::vector<double> calculateCollisionDisturb(
      Eigen::Matrix<double, 6, ARM_DOF>world_J_veh_tip,
      Eigen::Matrix4d wTpegtip, Eigen::Vector3d forces, Eigen::Vector3d torques);

  static std::vector<double> calculateCollisionDisturb(
      Eigen::Matrix<double, 6, ARM_DOF>world_J_veh_tip,
      Eigen::Matrix4d wTpegtip, Eigen::Vector3d forces);

private:
  // Disallow creating an instance of this object, it is useless since all functions are static
  CollisionPropagator();

  static Eigen::Matrix<double, ARM_DOF, 1> fromPegTipToWholeArm(
      Eigen::Matrix<double, 6, ARM_DOF>tip_J_link0_tip,
      Eigen::Vector3d forces, Eigen::Vector3d torques);
  static Eigen::Matrix<double, ARM_DOF, 1> fromPegTipToWholeArm(
      Eigen::Matrix<double, 6, ARM_DOF>tip_J_link0_tip,
      Eigen::Vector3d forces);
};

#endif // COLLISIONPROPAGATOR_H
