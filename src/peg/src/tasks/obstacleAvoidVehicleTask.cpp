#include "header/obstacleAvoidVehicleTask.h"

ObstacleAvoidVehicleTask::ObstacleAvoidVehicleTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "OBSTACLE_AVOID_VEH"){

  if (dimension != 1){
    std::cerr << "[" << robotName << "][" << taskName << "] ERROR, this task is intended to be a scalar one, "<<
                 "you setted "<< dimension << " as dimension\n";
    return;
  }
  gain = 0.6;
  safe_dist = 0.8; //in meters

}

int ObstacleAvoidVehicleTask::updateMatrices(struct Infos* const robInfo){

  //distance in world frame between the base frame of the 2 robot
  Eigen::Vector3d w_dist = robInfo->exchangedInfo.otherRobPos -
      robInfo->robotState.wTv_eigen.topRightCorner<3,1>();

  // if norm is too little we cant provide a ref because will tend to infinite.
  // However, if norm is so little the robot are already compenetrating so gain safe dist
  // and veh_dim are too little
  if (w_dist.norm() < 0.00001) {
    std::cout << "[" << robotName << "][" << taskName << "] ERROR: Norm too little,"
              << "not computing things" << std::endl;
    return -1;
  }

  setActivation(w_dist);
  A.PrintMtx("A");

  setJacobian(robInfo->robotState.wTv_eigen, w_dist);
  J.PrintMtx("J");

  setReference(w_dist);
  reference.PrintMtx("REF");


  return 0;
}

void ObstacleAvoidVehicleTask::setActivation(Eigen::Vector3d w_dist){

  //we know it is a scalar task
  A(1) = CMAT::DecreasingBellShapedFunction(VEH_DIM, VEH_DIM + safe_dist, 0, 1, w_dist.norm());

}

void ObstacleAvoidVehicleTask::setJacobian(Eigen::Matrix4d wTv, Eigen::Vector3d w_dist){

  Eigen::Matrix<double, 1, TOT_DOF> jacobian_eigen = Eigen::Matrix<double, 1, TOT_DOF>::Zero();
  //first ARM_DOF column zero

  //linear part
  Eigen::Vector3d w_dist_normal = w_dist / w_dist.norm();

  jacobian_eigen.block<1,3>(0,4) =
       -w_dist_normal.transpose() * wTv.topLeftCorner<3,3>();

  //angular part vehicle zero


  J = CONV::matrix_eigen2cmat(jacobian_eigen);

}

void ObstacleAvoidVehicleTask::setReference(Eigen::Vector3d w_dist){

  //TODO check where the frame is to undersand well distance minimum

  //if the frame is in the middle, distance minim is two times the half of lenght
  //(plus the safe dist)
  //(1): we know this is a scalar task
  this->reference(1) = gain * (VEH_DIM + safe_dist) - w_dist.norm();

}


