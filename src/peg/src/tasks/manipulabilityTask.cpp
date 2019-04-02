#include "header/manipulabilityTask.h"

ManipulabilityTask::ManipulabilityTask(int dim, bool eqType)
  : Task(dim, eqType, "MANIPULABILITY"){
  gain = 0.2;

  mu = 0.0;
}


int ManipulabilityTask::updateMatrices(struct Infos* const robInfo){

  setActivation();
  setJacobian();
  setReference();
  return 0;

}

int ManipulabilityTask::setJacobian(){
  //CASINO mi serve la derivata della jacobiana
}


/**
 * @brief ManipulabilityTask::setReference Value taken from matlab code for robust es
 */
void ManipulabilityTask::setReference(){
  reference = gain * (0.12 - mu)

}

/**
 * @brief ManipulabilityTask::setActivation If mu <0.02 A=1, else if mu>0.05 A=0.
 * In between there is a smooth behaviour
 */
void ManipulabilityTask::setActivation(){
  A = CMAT::DecreasingBellShapedFunction(0.02, 0.05, 0, 1, mu);
}
