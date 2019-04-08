#ifndef INFOS_H
#define INFOS_H

#include "../../support/header/defines.h"
#include <Eigen/Core>
#include <vector>

/**
 * @brief The Transforms struct
 * VARIABLE AND CONSTANT infos necessary to the mission, but not related directly to robot
 * (e.g. position of a goal, control point in the world (tool point))
 */
struct Transforms {

  Eigen::Matrix4d wTgoalVeh_eigen; //goal for vehicle
  Eigen::Matrix4d wTgoalEE_eigen; //goal for EE (will be projected on veh inside task)
  Eigen::Matrix4d wTgoalTool_eigen; //goal for tool (tool frame is in the center of pipe)

  Eigen::Matrix4d wTt_eigen; //tool in the world

};

/**
 * @brief The RobotState struct
 * VARIABLE info about the robot (e.g. position of vehicle respect world)
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

  /// vehicle and joint and Sensors state
  std::vector<double> vehicleVel;
  std::vector<double> jState; //joint state (rad)

  /// Jacobians

  //geometric jacobian of the arm projected on link 0 with end effector as last frame
 // (ie: ee is the controlled one)
  Eigen::Matrix<double, 6, ARM_DOF> link0_Jee_man;

  //geometric jacobian of the arm projected on link 0 with tool as last frame
 // (ie: tool is the controlled one)
  Eigen::Matrix<double, 6, ARM_DOF> link0_Jtool_man;

  //geometric jacobian of the whole robot projected on the world
  //with end effector as last frame (ie: ee is the controlled one)
  //(J=[J_pos ; J_or] from Antonelly book
  Eigen::Matrix<double, 6, TOT_DOF> w_Jee_robot;
  //geometric jacobian of the whole robot projected on the world
  //with tool as last frame (ie: tool is the controlled one)
  //(J=[J_pos ; J_or] from Antonelly book
  Eigen::Matrix<double, 6, TOT_DOF> w_Jtool_robot;



};

/**
 * @brief The RobotStruct struct
 * CONSTANT info about the robot (e.g. fixed frame for vehicle to sensors)
 */
struct RobotStruct {

 //Necessary to compute each time the vTee in the control loop.
 Eigen::Matrix4d  vTlink0; //transformation between vehicle and link 0 (so, a fixed one)

};

/**
 * @brief The ExchangedInfo struct
 * VARIABLE infos about thing not physically internally in the robot
 * (e.g position of other robot)
 * @warning It is very important to keep this info as small as possible, because
 * exchanging infos underwater is slow and difficult: only strictful needed info here
 * (e.g. a 3d vector instead of whole trasf matrix if only the pos is needed)
 */
struct ExchangedInfo {
  //position in the world of the other twin robot. for now used only for obstacledistancetask
  //(where the obstacle is the other vehicle)
  Eigen::Vector3d otherRobPos;
};

struct Infos {
  RobotStruct robotStruct;
  RobotState robotState;
  Transforms transforms;
  ExchangedInfo exchangedInfo;
};

#endif // INFOS_H
