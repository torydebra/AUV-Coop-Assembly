#include "header/collisionPropagator.h"

CollisionPropagator::CollisionPropagator(){} //private costructor not to be used


std::vector<double> CollisionPropagator::calculateCollisionDisturb(Eigen::Matrix<double, 6, ARM_DOF>world_J_veh_tip,
                                                Eigen::Matrix4d wTpegtip, Eigen::Vector3d forces, Eigen::Vector3d torques){

  Eigen::Matrix<double, 6, ARM_DOF>tip_J_veh_tip;
  Eigen::Matrix3d pegtipRw = wTpegtip.topLeftCorner<3,3>().transpose();
  tip_J_veh_tip.topRows<3>() = pegtipRw * world_J_veh_tip.topRows<3>();
  tip_J_veh_tip.bottomRows<3>() = pegtipRw * world_J_veh_tip.bottomRows<3>();

  Eigen::Matrix<double, ARM_DOF, 1> deltayDot_eigen =
      CollisionPropagator::fromPegTipToWholeArm(tip_J_veh_tip, forces, torques);

  return CONV::vector_eigen2std(deltayDot_eigen);

}

std::vector<double> CollisionPropagator::calculateCollisionDisturbVeh(Eigen::Matrix<double, 6, VEHICLE_DOF> world_J_veh_tip,
                                                                      Eigen::Matrix4d wTpegtip, Eigen::Vector3d forces,
                                                                      Eigen::Vector3d torques){

  Eigen::Matrix<double, 6, VEHICLE_DOF>tip_J_veh_tip;
  Eigen::Matrix3d pegtipRw = wTpegtip.topLeftCorner<3,3>().transpose();
  tip_J_veh_tip.topRows<3>() = pegtipRw * world_J_veh_tip.topRows<3>();
  tip_J_veh_tip.bottomRows<3>() = pegtipRw * world_J_veh_tip.bottomRows<3>();

  Eigen::Matrix<double, VEHICLE_DOF, 1> deltayDotVeh_eigen =
      CollisionPropagator::fromPegTipToVehicle(tip_J_veh_tip, forces, torques);

  return CONV::vector_eigen2std(deltayDotVeh_eigen);

}



std::vector<double> CollisionPropagator::calculateCollisionDisturb(Eigen::Matrix<double, 6, ARM_DOF>world_J_veh_tip,
                                                Eigen::Matrix4d wTpegtip, Eigen::Vector3d forces){

  Eigen::Matrix<double, 6, ARM_DOF>tip_J_veh_tip;
  Eigen::Matrix3d pegtipRw = wTpegtip.topLeftCorner<3,3>().transpose();
  tip_J_veh_tip.topRows<3>() = pegtipRw * world_J_veh_tip.topRows<3>();
  tip_J_veh_tip.bottomRows<3>() = pegtipRw * world_J_veh_tip.bottomRows<3>();

  Eigen::Matrix<double, ARM_DOF, 1> deltayDot_eigen =
      CollisionPropagator::fromPegTipToWholeArm(tip_J_veh_tip, forces);

  return CONV::vector_eigen2std(deltayDot_eigen);

}






/**
 * @brief CollisionPropagation::fromPegTipToWholeArm
 * @param tip_J_veh_tip jacobian from vehicle to peg tip PROJECTED on peg tip
 * @param forces the force x y z caused by collision
 * @param torques the torques x y z caused by collision
 * @return deltayDot the disturb on each joint caused by the collision
 * @note the formula: deltaYdot = JacobianLinear^T * forces + JacobianLinear^T * torques.
 *  where deltaYdot is the command-disturb that will ust be added at the command output of
 *  tpik. jacobian is the one from vehicle to peg tip PROJECTED on peg tip
 *  and force and torques projected on peg tip (already provided by sensor projected here)
 * @warning give the jacobian PROJECTED on peg tip! (pegtipRw * w_Jtool_robot)
 */
Eigen::Matrix<double, ARM_DOF, 1> CollisionPropagator::fromPegTipToWholeArm(
    Eigen::Matrix<double, 6, ARM_DOF>tip_J_veh_tip, Eigen::Vector3d forces, Eigen::Vector3d torques){

  Eigen::Matrix<double, ARM_DOF, 1> deltayDot_eigen;

  deltayDot_eigen = ( (tip_J_veh_tip.topRows<3>().transpose()) * forces ) +
                    ( (tip_J_veh_tip.bottomRows<3>().transpose()) * torques );

  return deltayDot_eigen;
}

/**
 * @brief CollisionPropagator::fromPegTipToVehicle calculate collision effects on the vehicle
 * @param tip_J_veh_tip
 * @param forces
 * @param torques
 * @return
 */
Eigen::Matrix<double, VEHICLE_DOF, 1> CollisionPropagator::fromPegTipToVehicle(
    Eigen::Matrix<double, 6, VEHICLE_DOF>tip_J_veh_tip, Eigen::Vector3d forces, Eigen::Vector3d torques){

  Eigen::Matrix<double, VEHICLE_DOF, 1> deltayDotVeh_eigen;

  deltayDotVeh_eigen = ( (tip_J_veh_tip.topRows<3>().transpose()) * forces ) +
                       ( (tip_J_veh_tip.bottomRows<3>().transpose()) * torques );

  return deltayDotVeh_eigen;



}


/**
 * @brief CollisionPropagation::fromPegTipToWholeArm
 * @param tip_J_veh_tip jacobian from vehicle to peg tip PROJECTED on peg tip
 * @param forces the force x y z caused by collision
 * @return deltayDot the disturb on each joint caused by the collision only considering forces
 * @note the formula: deltaYdot = JacobianLinear^T * forces + JacobianLinear^T * torques.
 *  where deltaYdot is the command-disturb that will ust be added at the command output of
 *  tpik. jacobian is the one from vehicle to peg tip PROJECTED on peg tip
 *  and force and torques projected on peg tip (already provided by sensor projected here)
 * @warning give the jacobian PROJECTED on peg tip! (pegtipRw * w_Jtool_robot)

 */
Eigen::Matrix<double, ARM_DOF, 1> CollisionPropagator::fromPegTipToWholeArm(
    Eigen::Matrix<double, 6, ARM_DOF>tip_J_veh_tip, Eigen::Vector3d forces){

  Eigen::Matrix<double, ARM_DOF, 1> deltayDot_eigen;

  deltayDot_eigen = ( (tip_J_veh_tip.topRows<3>().transpose()) * forces );

  return deltayDot_eigen;
}
