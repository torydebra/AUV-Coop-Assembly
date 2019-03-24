#include "header/controller.h"

/**
 * @brief Controller::Controller Default constructor
 * It creates the tasks (with new) and add them to the list of task
 *
 *
 * @note   note: std::vector is nicer because to change the order of priority or to leave for the moment
 * a task we can simply comment the row.
 * instead, with tasks as Task**, we need to fill the list with task[0], ...task[i] ... and changing
 * priority order would be slower.
 *
 */
Controller::Controller() {

  //tasks = new Task*[NUM_TASKS]; //not necessary with tasks std::vector

  /// PUT HERE NEW TASKS. FOR THIS CLASS, ONLY MODIFICATIONS HERE ARE NECESSARY
  // note: order of priority at the moment is here
  bool eqType = true;
  bool ineqType = false;
  tasks.push_back(new VehicleReachTask(6, TOT_DOF, eqType));

  // store number of task inserted
  numTasks = tasks.size();
}


///   TO NOT MODIFY BELOW TO ADD NEW TASK

/** @brief Controller::~Controller Default destructor
 * @note It is important to delete singularly all object pointed by the vector tasks. simply tasks.clear()
 * deletes the pointer but not the object Task pointed
*/
Controller::~Controller(){
  for (std::vector< Task* >::iterator it = tasks.begin() ; it != tasks.end(); ++it)
    {
      delete (*it);
    }
    tasks.clear();
}

/**
 * @brief Controller::updateTransforms This function calls all the updateMatrices of each task inserted in the
 * costructor.
 * @param transf the struct where all infos needed by all the task are
 * @return 0 to correct execution
 * @note usage of overridden pure virtual method updateMatrices
 */
int Controller::updateTransforms(struct Transforms* const transf){

  for (int i=0; i<numTasks; i++){
    tasks[i]->updateMatrices(transf);
  }

  return 0;
}

/**
 * @brief Controller::execAlgorithm this function calls the step of the algorithm modifying (in the for)
 * each time the projection matrix Q and command vector qDot.
 * The function call equalityIcat or inequalityIcat according to the type of task.
 * @return the qDot command to be send to the robot at each loop
 *
 */
std::vector<double> Controller::execAlgorithm(){

  //initialize qdot and Q for algorithm
  //qdot = [arm arm arm arm wx wy wz x y z]
  CMAT::Matrix qDot_cmat = CMAT::Matrix (TOT_DOF,1);
  CMAT::Matrix Q = CMAT::Matrix::Eye(TOT_DOF);

  for (int i=0; i<numTasks; i++){
    if (tasks[i]->eqType){
      Controller::equalityIcat(tasks[i], &qDot_cmat, &Q);
    } else {
      Controller::inequalityIcat(tasks[i], &qDot_cmat, &Q);
    }
  }

  std::vector<double> qDot_vect(TOT_DOF);
  int i = 1;
  for(std::vector<double>::iterator it = qDot_vect.begin(); it != qDot_vect.end(); ++it) {
    *it = qDot_cmat(i);
    i++;
  }

  return qDot_vect;

}

/**
 * @brief Controller::equalityIcat icat algorithm taken from another code. It uses the regolarized pseudoinverse
 * of cmat library.
 * @param task for convenience, a pointer to the task is passed even if the class has
 * the list of task std::vector<Task*> tasks as member. Anyway, it is only a pointer, no copy of the heavy
 * object task is performed.
 * @param rhop the "temporary" command qdot which will be modified by succesively calls of Icat
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
 * @param rhop the "temporary" command qdot which will be modified by succesively calls of Icat
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

