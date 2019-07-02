#include "header/controller.h"

/**
 * @brief Controller::Controller constructor
 * @param name of the robot (for printing things)
 */
Controller::Controller(std::string robotName) {
  this->robotName = robotName;
  std::cout << "[" << robotName<< "][CONTROLLER] Start " << std::endl;
}


/**
 * @brief Controller::updateAllTaskMatrices This function calls all the updateMatrices of each task
 * @param tasks the vector of tasks pointer
 * @param robInfo the struct where all infos needed by all the tasks are
 * @return 0 to correct execution
 * @note usage of overridden pure virtual method tasks[i]->updateMatrices()
 */
int Controller::updateMultipleTasksMatrices(std::vector<Task*> tasks, struct Infos* const robInfo){

  int numTasks = tasks.size();
  for (int i=0; i<(numTasks-1); i++){ //LastTask has everything fixed
    if (tasks[i]->updated == false){
      tasks[i]->updateMatrices(robInfo);
      tasks[i]->updated = true;
//      std::cout << "[" << robotName<< "][CONTROLLER] Updated transforms for " <<
//                   tasks[i]->getName() << std::endl;

    }
  }

  return 0;
}

/**
 * @brief Controller::updateSingleTaskMatrices This function calls the updateMatrices for a single task
 * @param *task pointer to the task that must be updated
 * @param robInfo the struct where all infos needed by all the tasks are
 * @return 0 to correct execution
 * @note usage of overridden pure virtual method task->updateMatrices()
 */
int Controller::updateSingleTaskMatrices(Task* task, struct Infos* const robInfo){

  if (task->updated == false){
    task->updateMatrices(robInfo);
    task->updated = true;
    std::cout << "[" << robotName<< "][CONTROLLER] Calculated transforms for only the " <<
                 task->getName() <<" task" << std::endl;
  } else {
    std::cout << "[" << robotName<< "][CONTROLLER] " <<
                 task->getName() <<" task already updated" << std::endl;
  }
  return 0;
}

/**
 * @brief Controller::resetAllUpdatedFlags to be called at the end of the control loop,
 * so next loop all matrices are updated (because robot moved)
 * @param tasks list of task to reset
 * @return 0 correct exec
 */
int Controller::resetAllUpdatedFlags(std::vector<Task*> tasks){

  int numTasks = tasks.size();
  for (int i=0; i<(numTasks-1); i++){ //LastTask has everything fixed
    tasks[i]->updated = false;
  }
//  std::cout << "[" << robotName<< "][CONTROLLER] Resetted Updated Flags for " <<
//               numTasks-1 <<" (exlcuding the null final task) tasks" << std::endl;
  return 0;

}

/**
 * @brief Controller::resetAllAlgosFlag Flag_W Mu_W Flag_G Mu_G must be resetted before each TPIK
 * @return
 */
int Controller::resetAllAlgosFlag(std::vector<Task*> tasks){

  int numTasks = tasks.size();
  for (int i=0; i<(numTasks); i++){
    tasks[i]->setFlag_G(0);
    tasks[i]->setFlag_W(0);
    tasks[i]->setMu_G(0.0);
    tasks[i]->setMu_W(0.0);

  }
//  std::cout << "[" << robotName<< "][CONTROLLER] Resetted Updated Flags for " <<
//               numTasks-1 <<"+1(the null task) tasks" << std::endl;
  return 0;
}

/**
 * @brief Controller::execAlgorithm this function calls the step of the algorithm modifying (in the for)
 * each time the projection matrix Q and command vector yDot.
 * The function call equalityIcat or inequalityIcat according to the type of task.
 * @return the yDot command to be send to the robot at each loop
 *
 */
std::vector<double> Controller::execAlgorithm(std::vector<Task*> tasks){

  Controller::resetAllAlgosFlag(tasks);

  //initialize yDot and Q for algorithm
  //yDot = [arm arm arm arm wx wy wz x y z]
  CMAT::Matrix yDot_cmat = CMAT::Matrix::Zeros(TOT_DOF,1);
  CMAT::Matrix Q = CMAT::Matrix::Eye(TOT_DOF);
  for (int i=0; i<tasks.size(); i++){

    if (tasks[i]->eqType){
      //std::cout<<tasks[i]->gain<<"\n";  ///DEBUG
      Controller::equalityIcat(tasks[i], &yDot_cmat, &Q);

    } else {
      Controller::inequalityIcat(tasks[i], &yDot_cmat, &Q);
    }

    ////DEBUG
//    std::cout << "[CONTROLLER] yDot after " << tasks[i]->getName() << ": \n";
//    yDot_cmat.PrintMtx();
//    std::cout << "\n";
  }

  return (CONV::vector_cmat2std(yDot_cmat));

}



/**
 * @brief Controller::equalityIcat icat algorithm taken from another code. It uses the regolarized pseudoinverse
 * of cmat library.
 * @param *task the pointer to the actual task that is being considered
 * @param rhop the "temporary" command yDot which will be modified by succesively calls of Icat
 * @param Q the prokection matrix which will be modified by succesively calls of Icat
 * @return 0 to correct execution
 *
 * @note mu and flag are variable needed by cmat pseudoinverse.
 * In cmat library they are labelled as "out" variables because the functions there modify them.
 * Note that they are two couples: one for calculate W matrix, the other for barGpinv
 * @note even though this function is a special case of inequality (with Activation identity matrix) it is
 * convenient to gain computation speed.
 */
int Controller::equalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) {

  CMAT::Matrix J = task->getJacobian(); //J task jacobian
  CMAT::Matrix ref = task->getReference(); //task reference

  CMAT::Matrix I = CMAT::Matrix::Eye(task->getDof());
  CMAT::Matrix barG, barGtransp, T, W, barGpinv;
  // barG the actual Jacobian
  barG = J * (*Q);
  barGtransp = barG.Transpose();

  /// Regularization matrices
  // T is the reg. matrix for the different levels of task priority
  // takes into account that not all the controls are available
  T = (I - (*Q)).Transpose() * (I-(*Q));

  // compute W to solve the problem of discontinuity due to different priority levels
  W = barG *
      (barGtransp * barG + T)
        .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                          task->getMu_W(), task->getFlag_W()) *
      barGtransp;

  /// Compute the rho for this task
  barGpinv = (barGtransp * barG)
      .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                        task->getMu_G(), task->getFlag_G());
  (*rhop) = (*rhop) + (*Q) * barGpinv * barGtransp * W * (ref - J * (*rhop));

  ///TODO: in ctrl_task_algo calcola anche un tmpProjector e una P_ che non so a che servono


  /// Update the projector matrix
  (*Q) = (*Q) * (I - barGpinv * barGtransp * barG);

  return 0;

}
/**
 * @brief Controller::inequalityIcat icat algorithm taken from another code. It uses the regolarized pseudoinverse
 * of cmat library.
 * @param *task the pointer to the actual task that is being considered
 * @param rhop the "temporary" command yDot which will be modified by succesively calls of Icat
 * @param Q the prokection matrix which will be modified by succesively calls of Icat
 * @return 0 to correct execution
 *
 * @note mu and flag are variable needed by cmat pseudoinverse.
 * In cmat library they are labelled as "out" variables because the functions there modify them.
 * Note that they are two couples: one for calculate W matrix, the other for barGpinv
 * @note this is the inequality version, where more computations are necessary.
 */
int Controller::inequalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) {

  CMAT::Matrix J = task->getJacobian(); //Jacobiana of task
  CMAT::Matrix ref = task->getReference(); // Reference of task
  CMAT::Matrix A = task->getActivation(); // Activation of task

  CMAT::Matrix I = CMAT::Matrix::Eye(task->getDof());
  CMAT::Matrix barG, barGtransp, barGtranspAA, T, H, W, barGpinv;
  // barG the actual Jacobian
  barG = J * (*Q);
  barGtransp = barG.Transpose();
  barGtranspAA = barGtransp * A * A;

  /// Regularization matrices
  // T is the reg. matrix for the different levels of task priority
  // takes into account that not all the controls are available
  T = (I - (*Q)).Transpose() * (I-(*Q));
  // H is the reg. matrix for the activation, i.e. A*(I-A)
  H = barGtransp *
      (CMAT::Matrix::Eye(task->getDimension()) - A)*
      A * barG;

  // compute W to solve the problem of discontinuity due to different priority levels
  W = barG *
      (barGtranspAA * barG + T + H)
        .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                          task->getMu_W(), task->getFlag_W()) *
      barGtranspAA;

  barGpinv = (barGtranspAA * barG + H)
               .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                                 task->getMu_G(), task->getFlag_G());


  ///TODO: in ctrl_task_algo calcola anche un tmpProjector e una P_ che non so a che servono


  /// Compute the rho for this task
  (*rhop) = (*rhop) + (*Q) * barGpinv * barGtranspAA * W * (ref - J * (*rhop));

  /// Update the projector matrix
  (*Q) = (*Q) * (I - barGpinv * barGtranspAA * barG);

  return 0;
}

