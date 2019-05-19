#include "header/vehicleReachTask.h"

/**
 * @brief VehicleReachTask::VehicleReachTask Constructor of specific task simply calls the parent constructor
 * through inizializer list
 * @param dimension dimension of the task (e.g. 1 for scalar task)
 * @param eqType true or false for equality or inequality task
 * @param activeLog bool to set logger prints
 */
VehicleReachTask::VehicleReachTask(int dim, bool eqType, std::string robotName, LinAngType linAngType)
  : Task(dim, eqType, robotName, "VEHICLE_REACH_GOAL") {
  gain = 0.2;
  this->linAngType = linAngType;
}

/**
 * @brief VehicleReachTask::updateMatrices overriden of the pure virtual method of Task parent class
 * @param robInfo struct filled with all infos needed by the task to compute the matrices
 * @return 0 for correct execution
 */
int VehicleReachTask::updateMatrices(struct Infos* const robInfo){

  setActivation();
  setJacobian(robInfo->robotState.wTv_eigen);
  setReference(robInfo->robotState.wTv_eigen, robInfo->transforms.wTgoalVeh_eigen);
  return 0;
}

int VehicleReachTask::setJacobian(Eigen::Matrix4d wTv_eigen){

  Eigen::Matrix<double, 3, TOT_DOF> jacobianLIN_eigen = Eigen::Matrix<double, 3, TOT_DOF>::Zero();
  Eigen::Matrix<double, 3, TOT_DOF> jacobianANG_eigen = Eigen::Matrix<double, 3, TOT_DOF>::Zero();

  Eigen::Matrix3d wRv_eigen = wTv_eigen.topLeftCorner(3,3);

  //matrix([1:6];[1:4]) joint part must be zero
  //matrix([1:3];[5:7]) linear part
  if (linAngType == LINEAR || linAngType == LIN_ANG || linAngType == YAXIS){
    jacobianLIN_eigen.block<3,3>(0,4) = wRv_eigen; //block: <dimensions> & (beginning row, beginning column)
  }
  //matrix([4:6];[8:10]) angular
  //according to eigen doc, using these kind of specific function (and not
  //.block) improves performance
  if (linAngType == ANGULAR || linAngType == LIN_ANG){
    jacobianANG_eigen.bottomRightCorner(3,3) = wRv_eigen;
  }

  //to cmat
  switch (linAngType){
  case LINEAR:
    this->J = CMAT::Matrix(dimension, dof, jacobianLIN_eigen.data());
    break;
  case ANGULAR:
    this->J = CMAT::Matrix(dimension, dof, jacobianANG_eigen.data());
    break;
  case YAXIS:
    this->J = CONV::matrix_eigen2cmat(jacobianLIN_eigen.row(1));
    break;
  case LIN_ANG:
    Eigen::Matrix<double, 6, TOT_DOF> jacobian_eigen = Eigen::Matrix<double, 6, TOT_DOF>::Zero();
    jacobian_eigen.topRows(3) = jacobianLIN_eigen;
    jacobian_eigen.bottomRows(3) = jacobianANG_eigen;
    this->J = CONV::matrix_eigen2cmat(jacobian_eigen);
    break;

  }

  //J.PrintMtx("JACOB");  ///DEBUG

}

int VehicleReachTask::setActivation(){

  double vectDiag[dimension];
  std::fill_n(vectDiag, dimension, 1);
  this->A.SetDiag(vectDiag);

  return 0;
}

int VehicleReachTask::setReference(
    Eigen::Matrix4d wTv_eigen, Eigen::Matrix4d wTg_eigen){

  CMAT::TransfMatrix wTv_cmat = CONV::matrix_eigen2cmat(wTv_eigen);
  CMAT::TransfMatrix wTg_cmat = CONV::matrix_eigen2cmat(wTg_eigen);
  CMAT::Vect6 errorSwapped = CMAT::CartError(wTg_cmat, wTv_cmat); //ang,lin


  // ang and lin must be swapped because in yDot and jacob the linear part is before
  switch (linAngType){
  case LINEAR:
    error(1) = errorSwapped(4);
    error(2) = errorSwapped(5);
    error(3) = errorSwapped(6);
    break;
  case ANGULAR:
    error(1) = errorSwapped(1);
    error(2) = errorSwapped(2);
    error(3) = errorSwapped(3);
    break;
  case LIN_ANG:
    error(1) = errorSwapped(4);
    error(2) = errorSwapped(5);
    error(3) = errorSwapped(6);
    error(4) = errorSwapped(1);
    error(5) = errorSwapped(2);
    error(6) = errorSwapped(3);
  case YAXIS:
    error(1) = errorSwapped(5);
    break;
  }

  this->reference = this->gain * (this->error);
}
