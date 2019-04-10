#include "header/armShapeTask.h"

ArmShapeTask::ArmShapeTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "ARM_SHAPE"){
  gain = 0.1;

}

int ArmShapeTask::updateMatrices(struct Infos* const robInfo){

  std::vector<double> jointGoal(4);
  //set preferred goal at middle pos for each joint
  jointGoal[0] = (JLIM1_MAX - JLIM1_MIN)/2;
  jointGoal[1] = (JLIM2_MAX - JLIM2_MIN)/2;
  jointGoal[2] = (JLIM3_MAX - JLIM3_MIN)/2;
  jointGoal[3] = (JLIM4_MAX - JLIM4_MIN)/2;

  setActivation();
  setJacobian();
  setReference(jointGoal, robInfo->robotState.jState);
  return 0;

}

/**
 * @brief EndEffectorReachTask::setActivation
 * always active? could be ok see that this is an optimization task,
 * differtly from the joint limit one
 * @return
 */
int ArmShapeTask::setActivation(){

  double vectDiag[dimension];
  std::fill_n(vectDiag, dimension, 1);
  this->A.SetDiag(vectDiag);

  return 0;

}

int ArmShapeTask::setJacobian(){

  CMAT::Matrix eye = CMAT::Matrix::Eye(dimension);
  CMAT::Matrix zero = CMAT::Matrix::Zeros(dimension,VEHICLE_DOF);

  CMAT::Matrix tot = eye.RightJuxtapose(zero);
  J = tot;

}

int ArmShapeTask::setReference(std::vector<double> jointGoal,
                               std::vector<double> jState){

  for (int i=0; i<dimension; i++){
      reference(i+1) = gain * (jointGoal[i] - jState[i]);
  }

  return 0;
}
