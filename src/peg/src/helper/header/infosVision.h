#ifndef INFOSVISION_H
#define INFOSVISION_H

#include "../../support/header/defines.h"
#include <Eigen/Core>

/**
 * @brief The Transforms struct
 * VARIABLE AND CONSTANT infos necessary to the mission, but not related directly to robot
 * (e.g. position of a goal, control point in the world (tool point))
 */
struct TransformsVisio {

  Eigen::Matrix4d wTgoalVeh_eigen; //goal for vehicle

  Eigen::Matrix4d wTt_eigen; //tool in the world
  Eigen::Matrix4d wTh_eigen; //hole in the world
  Eigen::Matrix4d wTh_estimated_eigen; //estimated position of the hole

};

/**
 * @brief The RobotState struct
 * VARIABLE info about the robot (e.g. position of vehicle respect world)
 */
struct RobotVisioState {

  /// Transforms
  Eigen::Matrix4d wTv_eigen; //world to vehicle

};

/**
 * @brief The RobotStruct struct
 * CONSTANT info about the robot (e.g. fixed frame for vehicle to sensors)
 */
struct RobotVisioStruct {

 Eigen::Matrix4d  vTcameraL; //transformation between vehicle and camera Left (so, a fixed one)
 Eigen::Matrix4d  vTcameraR;
 Eigen::Matrix4d  cLTcR; //from camera left to camera right, redundant but useful for stereo

};


struct InfosVision {
  RobotVisioStruct robotStruct;
  RobotVisioState robotState;
  TransformsVisio transforms;
};

#endif // INFOSVISION_H
