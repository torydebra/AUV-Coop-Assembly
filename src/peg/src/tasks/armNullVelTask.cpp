#include "header/armNullVelTask.h"

ArmNullVelTask::ArmNullVelTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "ARM_NULL_VEL") {
  //no Gain: non reactive task
}

int ArmNullVelTask::updateMatrices(struct Infos* const robInfo){

  setActivation();
  setJacobian();
  setReference();
  return 0;
}


int ArmNullVelTask::setJacobian(){

  Eigen::MatrixXd jacobian_eigen = Eigen::MatrixXd::Zero(dimension, dof);
  jacobian_eigen.leftCols(dimension) = Eigen::MatrixXd::Identity(dimension, dimension);

  this->J = CMAT::Matrix(dimension, dof, jacobian_eigen.data());


}

int ArmNullVelTask::setActivation(){

  double vectDiag[6];
  std::fill_n(vectDiag, 6, 1);
  this->A.SetDiag(vectDiag);

  return 0;
}

int ArmNullVelTask::setReference(){

  this->reference = CMAT::Matrix::Zeros(dimension, 1);
}
