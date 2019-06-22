#include "header/collisionPropagation.h"

CollisionPropagation::CollisionPropagation()
{

}

std::vector<double> CollisionPropagation::fromPegTipToWholeArm(
    Eigen::Matrix<double, 6, ARM_DOF>tip_J_link0_tip, Eigen::Vector3d forces, Eigen::Vector3d torques){

  std::vector<double> deltaqDot(ARM_DOF);










  return deltaqDot;
}
