#include "header/horizontalAttitudeTask.h"

HorizontalAttitudeTask::HorizontalAttitudeTask(int dim, bool eqType)
  : Task(dim, eqType, "HORIZONTAL_ATTITUDE"){

  if (dimension != 1){
    std::cerr << "[" << taskName << "] ERROR, this task is intended to be a scalar one, "<<
                 "you setted "<< dimension << " as dimension\n";
    return;
  }

  gain = 0.3;

}

int HorizontalAttitudeTask::updateMatrices(Infos* const robInfo){

  setPhi(robInfo->robotState.wTv_eigen);

  setActivation();
  setJacobian();
  setReference();

  return 0;
}

/**
 * @brief HorizontalAttitudeTask::setPhi calculate misalignment versor between k versor of the vehicle and k of the world
 * @param wTv_eigen requested Transformation from world to vehicle
 * @note both k versor (world and vehicle) point down (i.e. +1 is below surface, -1 above)
 */
void HorizontalAttitudeTask::setPhi(Eigen::Matrix4d wTv_eigen){

  Eigen::Vector3d kHat;
  kHat << 0, 0, 1; //generic k versor

  // k versor of the world projected on vehicle frame
  Eigen::Matrix3d wRv_eigen = wTv_eigen.topLeftCorner(3,3);
  Eigen::Vector3d v_kHat_w = wRv_eigen.transpose() * kHat;

  phi = FRM::reducedVersorLemma(v_kHat_w, kHat);

}

/**
 * @brief HorizontalAttitudeTask::setActivation Values taken from matlab coe auvsim
 */
void HorizontalAttitudeTask::setActivation(){

  // activation we know that is a scalar because dimension = 1
  A(1) = CMAT::IncreasingBellShapedFunction(0.025, 0.1, 0, 1, phi.norm());
}

void HorizontalAttitudeTask::setJacobian(){

  Eigen::Vector3d normalPhi;
  if (phi.norm() > 0){ //to not divide by zero
    normalPhi = phi/(phi.norm());
  } else{
    normalPhi << 0,0,0;
  }

  //first element are zero, actually J is a row
  Eigen::MatrixXd J_eigen(dimension, dof);
  Eigen::MatrixXd temp(3,VEHICLE_DOF);
  temp.topLeftCorner(3,3) = Eigen::Matrix3d::Zero();
  temp.topRightCorner(3,3) = Eigen::Matrix3d::Identity();


  J_eigen.topLeftCorner(dimension, ARM_DOF) = Eigen::RowVectorXd::Zero(ARM_DOF);
  J_eigen.topRightCorner(dimension, VEHICLE_DOF) = normalPhi.transpose() * temp;

  J = CONV::matrix_eigen2cmat(J_eigen);
}

void HorizontalAttitudeTask::setReference(){
  //We know reference is a scalar becase task is scalar
  reference(1) = -gain * phi.norm();

}
