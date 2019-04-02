#include "header/jointLimitTask.h"

JointLimitTask::JointLimitTask(int dim, bool eqType)
  : Task(dim, eqType, "JOINT_LIMIT"){
  gain = 0.5;

  safeGuardUp = new double[this->dimension];
  safeGuardUp[0] = JLIM1_MAX - 0.2;
  safeGuardUp[1] = JLIM2_MAX - 0.2;
  safeGuardUp[2] = JLIM3_MAX - 0.2;
  safeGuardUp[3] = JLIM4_MAX - 0.4;

  safeGuardLow = new double[this->dimension];
  safeGuardLow[0] = JLIM1_MIN + 0.25;
  safeGuardLow[1] = JLIM2_MIN + 0.25;
  safeGuardLow[2] = JLIM3_MIN + 0.25;
  safeGuardLow[3] = JLIM4_MIN + 0.4;

  //We set as reference the middle point between the two safe guards that is,
  //the point farther from both safe guard limits
  halfPoint = new double[this->dimension];
  for(int i=0; i< dimension; i++){
    halfPoint[i] = (safeGuardUp[i] + safeGuardLow[i])/2.0;
    //std::cout << "HALFPOINT : " << halfPoint << "\n";
  }

}
JointLimitTask::~JointLimitTask(){
  delete []safeGuardLow;
  delete []safeGuardUp;
  delete []halfPoint;
}

int JointLimitTask::updateMatrices(struct Infos* const robInfo){

  setActivation(robInfo->robotState.jState);
  setJacobian();
  setReference(robInfo->robotState.jState);
  return 0;

}

int JointLimitTask::setActivation(std::vector<double> jState){

  double jLimUP[] = {JLIM1_MAX, JLIM2_MAX, JLIM3_MAX, JLIM4_MAX};
  double jLimLOW[] = {JLIM1_MIN, JLIM2_MIN, JLIM3_MIN, JLIM4_MIN};

  for (int i=1; i<=dimension; i++){
    A(i,i) = CMAT::DecreasingBellShapedFunction(jLimLOW[i-1], safeGuardLow[i-1],
                                                0, 1, jState[i-1]) +
             CMAT::IncreasingBellShapedFunction(safeGuardUp[i-1], jLimUP[i-1],
                                                 0, 1, jState[i-1]);
  }
}


int JointLimitTask::setJacobian(){

  CMAT::Matrix eye = CMAT::Matrix::Eye(dimension);
  CMAT::Matrix zero = CMAT::Matrix::Zeros(dimension,VEHICLE_DOF);

  CMAT::Matrix tot = eye.RightJuxtapose(zero);
  J = tot;

}

/**
 * @brief JointLimitTask::setReference
 * @return 0 correct execution
 * @note We set as reference the middle point between the two safe guards that is,
 * the point farther from both safe guard limits
 */
int JointLimitTask::setReference(std::vector<double> jState){

  /// DEBUG
//  for (int i=0; i<4; i++){
//    std::cout << jState[i] << "\t";
//  }
//  std::cout << "\n";

  for (int i=0; i<dimension; i++){

    if (jState[i] <= halfPoint[i]){
      reference(i+1) = gain * (safeGuardLow[i] - jState[i]);

    } else {
      reference(i+1) = gain * (safeGuardUp[i] - jState[i]);


    }
  }
  return 0;
}
