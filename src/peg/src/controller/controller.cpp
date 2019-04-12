#include "header/controller.h"

/**
 * @brief Controller::Controller constructor
 * @param name of the robot (for printing things)
 */
Controller::Controller(std::string robotName) {
  this->robotName = robotName;
}

/**
 * @brief Controller::setTaskList store the list of task that the controller must
 * consider.
 * @param tasks std::vector<Task*> list of tasks
 * @return 0 correct execution
 * @note only the vector of pointers is copied. Each specialized xxxTask is not copied, so we don't waste space
 */
int Controller::setTaskList(std::vector<Task*> tasks){
  this->tasks = tasks;
  // store number of task inserted
  numTasks = tasks.size();
  std::cout << "[" << robotName<< "][CONTROLLER] Inserted " <<
               numTasks-1 <<"+1(the null task) tasks" << std::endl;
}


/**
 * @brief Controller::updateTransforms This function calls all the updateMatrices of each task inserted in the
 * costructor.
 * @param robInfo the struct where all infos needed by all the tasks are
 * @return 0 to correct execution
 * @note usage of overridden pure virtual method updateMatrices
 */
int Controller::updateTransforms(struct Infos* const robInfo){

  for (int i=0; i<(numTasks-1); i++){ //LastTask has everything fixed
    tasks[i]->updateMatrices(robInfo);
  }
  return 0;
}

/**
 * @brief Controller::execAlgorithm this function calls the step of the algorithm modifying (in the for)
 * each time the projection matrix Q and command vector yDot.
 * The function call equalityIcat or inequalityIcat according to the type of task.
 * @return the yDot command to be send to the robot at each loop
 *
 */
std::vector<double> Controller::execAlgorithm(){

  //initialize yDot and Q for algorithm
  //yDot = [arm arm arm arm wx wy wz x y z]
  CMAT::Matrix yDot_cmat = CMAT::Matrix::Zeros(TOT_DOF,1);
  CMAT::Matrix Q = CMAT::Matrix::Eye(TOT_DOF);
  //std::cout << "eereer\n\n\n"; ///DEBUG
  for (int i=0; i<numTasks; i++){

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

  //TODO metterlo nel CONV
  std::vector<double> yDot_vect(TOT_DOF);
  int i = 1;
  for(std::vector<double>::iterator it = yDot_vect.begin(); it != yDot_vect.end(); ++it) {
    *it = yDot_cmat(i);
    i++;
  }

  return yDot_vect;

}

/**
 * @brief Controller::equalityIcat icat algorithm taken from another code. It uses the regolarized pseudoinverse
 * of cmat library.
 * @param task for convenience, a pointer to the task is passed even if the class has
 * the list of task std::vector<Task*> tasks as member. Anyway, it is only a pointer, no copy of the heavy
 * object task is performed.
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
 * @param task for convenience, a pointer to the task is passed even if the class has
 * the list of task std::vector<Task*> tasks as member. Anyway, it is only a pointer, no copy of the heavy
 * object task is performed.
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

