#include "header/obstacleAvoidVehicleTask.h"

ObstacleAvoidVehicleTask::ObstacleAvoidVehicleTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "OBSTACLE_AVOID_VEH"){

  if (dimension != 1){
    std::cerr << "[" << robotName << "][" << taskName << "] ERROR, this task is intended to be a scalar one, "<<
                 "you setted "<< dimension << " as dimension\n";
    return;
  }
  gain = 0.7;
  safe_dist = 0.75; //in meters

}

int ObstacleAvoidVehicleTask::updateMatrices(struct Infos* const robInfo){

  //distance in world frame between the base frame of the 2 robot
  Eigen::Vector3d w_dist = robInfo->exchangedInfo.otherRobPos -
      robInfo->robotState.wTv_eigen.topRightCorner<3,1>();

  // if norm is too little we cant provide a ref because will tend to infinite;
  // so we set a minimu value possible
  // However, if norm is so little the robot are already compenetrating so problems are other:
  //gain, safe_dist, and veh_dim maybe are too little
  double w_distNorm = w_dist.norm();
  if (w_distNorm < 0.001) {
    std::cout << "[" << robotName << "][" << taskName << "] WARNING: Norm too little,"
              << " I will set it to 0.001" << std::endl;
    w_distNorm = 0.001;
  }

  //std::cout << w_distNorm << " DIST NORMA\n";

  setActivation(w_distNorm);
  //A.PrintMtx("A");

  Eigen::Vector3d w_dist_normal = w_dist / w_distNorm;
  setJacobian(robInfo->robotState.wTv_eigen, w_dist_normal);
  //J.PrintMtx("J");

  setReference(w_distNorm);
  //reference.PrintMtx("REF");


  return 0;
}

void ObstacleAvoidVehicleTask::setActivation(double w_distNorm){

  //we know it is a scalar task
  A(1) = CMAT::DecreasingBellShapedFunction(VEH_DIM, VEH_DIM + safe_dist, 0, 1, w_distNorm);

}

void ObstacleAvoidVehicleTask::setJacobian(Eigen::Matrix4d wTv, Eigen::Vector3d w_dist_normal){

  Eigen::Matrix<double, 1, TOT_DOF> jacobian_eigen = Eigen::Matrix<double, 1, TOT_DOF>::Zero();

  //first ARM_DOF column zero

  //linear part
  jacobian_eigen.block<1,3>(0,4) =
       -w_dist_normal.transpose() * wTv.topLeftCorner<3,3>();

  //angular part vehicle zero


  J = CONV::matrix_eigen2cmat(jacobian_eigen);

}

void ObstacleAvoidVehicleTask::setReference(double w_distNorm){

  //TODO check where the frame is to undersand well distance minimum

  //if the frame is in the middle, distance minim is two times the half of lenght
  //(plus the safe dist)
  //(1): we know this is a scalar task
  this->reference(1) = gain * ((VEH_DIM + safe_dist) - w_distNorm);

}


