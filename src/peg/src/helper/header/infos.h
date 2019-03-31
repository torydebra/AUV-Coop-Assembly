#ifndef INFOS_H
#define INFOS_H

#include "../../support/header/defines.h"
#include <Eigen/Core>
#include <vector>

/**
 * @brief The Transforms struct infos necessary to the mission, but not related directly to robot (e.g. position of a goal)
 */
struct Transforms {

  Eigen::Matrix4d wTgoalVeh_eigen; //goal for vehicle
  Eigen::Matrix4d wTgoalEE_eigen; //goal for EE (will be projected on veh inside task)


};

/**
 * @brief The RobotState struct VARIABLE info about the robot (e.g. position of vehicle respect world)
 */
struct RobotState {

  /// Transforms
  Eigen::Matrix4d wTv_eigen; //world to vehicle
  Eigen::Matrix4d link0Tee_eigen; //link0 to end effector
  Eigen::Matrix4d vTee_eigen; //vehicle to end effector (redundant but useful)

  // ASK Is it necessary? or only vTEE is needed?
  //Eigen::Matrix4d link0TEE_eigen; //from base joint to EE

  //std::vector<Eigen::Matrix4d> vTjoints; // vehicle to each joint
  //Eigen::Matrix4d vTee_eigen; //vehicle to endEffector

  /// Sensors state
  std::vector<double> jState; //joint state (rad)

  /// Jacobians

  Eigen::Matrix<double, 6, ARM_DOF> link0_J_man; //geometric jacobian of the arm projected on link 0

  //geometric jacobian of the whole robot projected on the world
  //(J=[J_pos ; J_or] from Antonelly book
  Eigen::Matrix<double, 6, TOT_DOF> w_J_robot;



};

/**
 * @brief The RobotStruct struct CONSTANT info about the robot (e.g. fixed frame for vehicle to sensors)
 */
struct RobotStruct {

 //Necessary to compute each time the vTee in the control loop.
 Eigen::Matrix4d  vTlink0; //transformation between vehicle and link 0 (so, a fixed one)

};

struct Infos {
  RobotStruct robotStruct;
  RobotState robotState;
  Transforms transforms;
};

#endif // INFOS_H
