#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include "defines.h"
#include <Eigen/Core>
#include <vector>

struct Transforms {

  Eigen::Matrix4d wTgoalVeh_eigen; //goal for vehicle
  Eigen::Matrix4d wTv_eigen; //world to vehicle
  std::vector<Eigen::Matrix4d> vTjoints; // vehicle to each joint
  Eigen::Matrix4d vTee_eigen; //vehicle to endEffector
  Eigen::Matrix4d wTgoalEE_eigen; //goal for EE (will be projected on veh inside task)

  std::vector<double> jState; //joint state (rad)


};

#endif
