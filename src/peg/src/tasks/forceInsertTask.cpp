#include "header/forceInsertTask.h"

ForceInsertTask::ForceInsertTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "FORCE_INSERTION"){
  gain = 0.1;

}

int ForceInsertTask::updateMatrices(Infos* const robInfo){

  setActivation(robInfo->robotSensor.forcePegTip, robInfo->robotSensor.torquePegTip);
  setJacobian(robInfo->robotState.w_Jtool_robot);
  setReference(robInfo->robotSensor.forcePegTip, robInfo->robotSensor.torquePegTip);
  return 0;

}

/**
 * @brief ForceInsertTask::setActivation
 * @param force
 * @param torque
 * @todo at the moment is equality task always active
 */
void ForceInsertTask::setActivation(Eigen::Vector3d force, Eigen::Vector3d torque){
  double vectDiag[dimension];
  std::fill_n(vectDiag, dimension, 1);
  this->A.SetDiag(vectDiag);
}

void ForceInsertTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot){

  if (dimension == 3){
    J = CONV::matrix_eigen2cmat(w_J_robot.topRows<3>());
  }
}

void ForceInsertTask::setReference(Eigen::Vector3d force, Eigen::Vector3d torque){

  if (dimension == 3){ //only force
    error = -1 * CONV::matrix_eigen2cmat(force);
    reference = gain * error; //-1 because it is 0 - force
    reference = FRM::saturateCmat(reference, 0.5);


  } else if(dimension == 6){
    CMAT::Vect3 force_cmat = CONV::matrix_eigen2cmat(force);
    CMAT::Vect3 torque_cmat = CONV::matrix_eigen2cmat(torque);
    error(1) = ( - force_cmat(1));
    error(2) = ( - force_cmat(2));
    error(3) = ( - force_cmat(3));
    error(4) = ( - torque_cmat(1));
    error(5) = ( - torque_cmat(2));
    error(6) = ( - torque_cmat(3));
    reference(1) = gain * ( - force_cmat(1));
    reference(2) = gain * ( - force_cmat(2));
    reference(3) = gain * ( - force_cmat(3));
    reference(4) = gain * ( - torque_cmat(1));
    reference(5) = gain * ( - torque_cmat(2));
    reference(6) = gain * ( - torque_cmat(3));
    reference = FRM::saturateCmat(reference, 0.5);


  } else {
    std::cerr << "[" << taskName << "] DIMENSION OF TASK IS WRONG\n";
  }


}


