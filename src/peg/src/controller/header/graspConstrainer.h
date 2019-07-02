#ifndef GRASPCONSTRAINER_H
#define GRASPCONSTRAINER_H

#include <Eigen/Core>
#include <cmat/cmat.h>

#include "../../support/header/defines.h"
#include "../../support/header/conversions.h"


/**
 * @brief The GraspConstrainer class
 *
 * Method to calculate the "Firm Grasp Constraint". If the end effector
 * goes away from the peg of the other robot, this function is needed to calculate
 * velocities that drive again the ee towards the other robot peg.
 * In practice it fakes a firm grasp assumptions, velocity generates can be seen
 * as the one provided by friction in real situation.
 *
 * Class with only static method, so no costructor is callable. In practice
 * this is used as namespace, but used the class so it is similar to collision prop
 */
class GraspConstrainer
{

public:

  /**
   * @brief calculateGraspVelocities_arm for effect of firm grasp only on the arm
   * @param w_J_man
   * @param otherEE_T_EE
   * @return
   */
  static std::vector<double> calculateGraspVelocities_arm(
      Eigen::Matrix<double, 6, ARM_DOF> w_J_man,
      Eigen::Matrix4d otherEE_T_EE);

  /**
   * @brief calculateGraspVelocities_armVeh
   *    for effect of firm grasp on the arm and on the vehicle
   * @param world_J_veh_ee
   * @param otherEE_T_EE
   * @return
   */
  static std::vector<double> calculateGraspVelocities_armVeh(
      Eigen::Matrix<double, 6, TOT_DOF> w_Jee_robot,
      Eigen::Matrix4d wTotherPeg, Eigen::Matrix4d wTee);
//  static std::vector<double> calculateGraspVelocities_armVeh(
//      Eigen::Matrix<double, 6, TOT_DOF> w_Jee_robot,
//      Eigen::Matrix4d otherEE_T_EE);


private :
    GraspConstrainer();
};

#endif // GRASPCONSTRAINER_H
