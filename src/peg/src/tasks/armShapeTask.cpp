#include "header/armShapeTask.h"

ArmShapeTask::ArmShapeTask(int dim, bool eqType, std::string robotName, ShapeType shapeType)
  : Task(dim, eqType, robotName, "ARM_SHAPE"){
  gain = 0.05;
  rangeAct = 0.5; //used in activation
  this->shapeType = shapeType;
  std::string eqineq = (eqType) ? "equality" : "inequality";

  std::cout << "[" << robotName << "][ARM_SHAPE]" << " ... as " <<
            eqineq << "task\n";

}

int ArmShapeTask::updateMatrices(struct Infos* const robInfo){

  std::vector<double> jointGoal(4);
  if (shapeType == MID_LIMITS){
  //set preferred goal at middle pos for each joint
    jointGoal[0] = (JLIM1_MAX + JLIM1_MIN)/2;
    jointGoal[1] = (JLIM2_MAX + JLIM2_MIN)/2;
    jointGoal[2] = (JLIM3_MAX + JLIM3_MIN)/2;
    jointGoal[3] = (JLIM4_MAX + JLIM4_MIN)/2;

  } else if(shapeType == PEG_GRASPED_PHASE){

    jointGoal[0] = 0;
    //jointGoal[1] = (JLIM2_MAX + JLIM2_MIN)/2;
    //jointGoal[2] = (JLIM3_MAX + JLIM3_MIN)/2;
    //jointGoal[3] = (JLIM4_MAX + JLIM4_MIN)/2;
    jointGoal[1] = 0.3744;
    jointGoal[2] = 0.6556;
    jointGoal[3] = 1.570;

    //TODO first jointGoal is different from robot which stay in front and behind
    //the peg, now it is at the actual pos (so no effect on it)
  }

  setActivation(jointGoal, robInfo->robotState.jState);
  setJacobian();
  setReference(jointGoal, robInfo->robotState.jState);
  return 0;

}

/**
 * @brief EndEffectorReachTask::setActivation
 * two types: eq (set always one)
 *            ineq (smooth trasition  \/  (both transition start in goal)
 * @return
 */
int ArmShapeTask::setActivation(std::vector<double> jointGoal, std::vector<double> jState){

  if (eqType){
    double vectDiag[dimension];
    std::fill_n(vectDiag, dimension, 1);
    this->A.SetDiag(vectDiag);

  } else {
    for (int i=1; i<=dimension; i++){

      A(i,i) =
          CMAT::DecreasingBellShapedFunction(
            jointGoal[i-1]-rangeAct, jointGoal[i-1], 0, 1, jState[i-1]) +
          CMAT::IncreasingBellShapedFunction(
            jointGoal[i-1], jointGoal[i-1]+rangeAct, 0, 1, jState[i-1]);
    }

  }

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
      error(i+1) = jointGoal[i] - jState[i];
      reference(i+1) = gain * (jointGoal[i] - jState[i]);
  }

  return 0;
}
