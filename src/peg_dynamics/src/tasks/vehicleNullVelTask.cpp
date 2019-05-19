#include "header/vehicleNullVelTask.h"

VehicleNullVelTask::VehicleNullVelTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "VEHICLE_NULL_VEL") {
  //no Gain: non reactive task
}

/**
 * @brief VehicleNull::updateMatrices overriden of the pure virtual method of Task parent class
 * @param robInfo struct filled with all infos needed by the task to compute the matrices
 * @return 0 for correct execution
 */
int VehicleNullVelTask::updateMatrices(struct Infos* const robInfo){

  setActivation();
  setJacobian(robInfo->robotState.wTv_eigen);
  setReference();
  return 0;
}

/**
 * @brief VehicleNullVelTask::setJacobian
 * @param wTv_eigen transformation from world to vehicle
 * @return 0 for correct exec
 * @note same identical function of vehicleReachTask, because the
 * jacobian are exactly the same
 */
int VehicleNullVelTask::setJacobian(Eigen::Matrix4d wTv_eigen){

  Eigen::MatrixXd jacobian_eigen = Eigen::MatrixXd::Zero(dimension, dof);
  Eigen::Matrix3d wRv_eigen = wTv_eigen.topLeftCorner(3,3);

  //matrix([1:6];[1:4]) joint part must be zero
  //matrix([1:3];[5:7]) linear part
  jacobian_eigen.block<3,3>(0,4) = wRv_eigen; //block: <dimensions> & (beginning row, beginning column)

  //matrix([4:6];[8:10]) angular
  //according to eigen doc, using these kind of specific function (and not
  //.block) improves performance
  jacobian_eigen.bottomRightCorner(3,3) = wRv_eigen;

  //NOTE: other part of the matrices are already zero becase all jacobians
  //are inizialized as zero

  //to cmat
  //eigen unroll to vector for cmat function
  this->J = CMAT::Matrix(dimension, dof, jacobian_eigen.data());

  //J.PrintMtx("JACOB");  ///DEBUG

}

int VehicleNullVelTask::setActivation(){

  double vectDiag[6];
  std::fill_n(vectDiag, 6, 1);
  this->A.SetDiag(vectDiag);

  return 0;
}

int VehicleNullVelTask::setReference(){

  this->reference = CMAT::Matrix::Zeros(dimension, 1);
}

